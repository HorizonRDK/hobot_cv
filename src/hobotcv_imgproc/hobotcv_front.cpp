// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "hobotcv_imgproc/hobotcv_front.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "utils.h"

namespace hobot_cv {

bool check_padding_area(uint32_t top,
                        uint32_t bottom,
                        uint32_t left,
                        uint32_t right) {
  if (top == 0 && bottom == 0 && left == 0 && right == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv padding"), "No padding area!");
    return false;
  } else if (top % 2 != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv padding"),
                 "Invalid top size: %d! Padding size must be even");
    return false;
  } else if (bottom % 2 != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv padding"),
                 "Invalid bottom size: %d! Padding size must be even");
    return false;
  } else if (left % 2 != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv padding"),
                 "Invalid left size: %d! Padding size must be even");
    return false;
  } else if (right % 2 != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv padding"),
                 "Invalid right size: %d! Padding size must be even");
    return false;
  }
  return true;
}

std::unique_ptr<char[]> hobotcv_constant_padding(const char *src,
                                                 const int &src_h,
                                                 const int &src_w,
                                                 uint32_t top,
                                                 uint32_t bottom,
                                                 uint32_t left,
                                                 uint32_t right,
                                                 uint8_t value) {
  // value转成yuv值
  uint8_t value_y = value;
  uint8_t value_u = 128;
  uint8_t value_v = 128;

  uint32_t dst_w = src_w + left + right;
  uint32_t dst_h = src_h + top + bottom;
  uint32_t dst_y_size = dst_w * dst_h;
  uint32_t dst_uv_size = dst_w * dst_h / 2;
  size_t dst_size = dst_y_size + dst_uv_size;
  std::unique_ptr<char[]> unique(new char[dst_size]);
  auto dst = unique.get();
  char *dst_y = dst;
  char *dst_uv = dst + dst_y_size;

  // padding y
  for (uint32_t h = 0; h < dst_h; ++h) {
    if (h < top || h >= src_h + top) {
      auto *raw = dst_y + h * dst_w;
      memset(raw, value_y, dst_w);
    } else {
      // padding left
      auto *raw = dst_y + h * dst_w;
      memset(raw, value_y, left);
      // copy src
      raw = dst_y + h * dst_w + left;
      auto *src_y_raw = src + (h - top) * src_w;
      memcpy(raw, src_y_raw, src_w);
      // padding right
      raw = dst_y + h * dst_w + left + src_w;
      memset(raw, value_y, right);
    }
  }

  // padding uv
  auto *src_uv_data = src + src_h * src_w;
  for (uint32_t h = 0; h < dst_h / 2; ++h) {
    auto *raw = dst_uv + h * dst_w;
    if ((h < (top / 2)) || (h >= (src_h + top) / 2)) {
      for (uint32_t w = 0; w < dst_w; w += 2) {
        *(raw + w) = value_u;
        *(raw + w + 1) = value_v;
      }
    } else {
      auto *raw = dst_uv + h * dst_w;
      for (uint32_t w = 0; w < left; w += 2) {
        *(raw + w) = value_u;
        *(raw + w + 1) = value_v;
      }
      raw = dst_uv + h * dst_w + left;
      auto *src_uv_raw = src_uv_data + (h - (top / 2)) * src_w;
      memcpy(raw, src_uv_raw, src_w);
      raw = dst_uv + h * dst_w + left + src_w;
      for (uint32_t w = 0; w < right; w += 2) {
        *(raw + w) = value_u;
        *(raw + w + 1) = value_v;
      }
    }
  }

  return unique;
}

std::unique_ptr<char[]> hobotcv_replicate_padding(const char *src,
                                                  const int &src_h,
                                                  const int &src_w,
                                                  uint32_t top,
                                                  uint32_t bottom,
                                                  uint32_t left,
                                                  uint32_t right) {
  uint32_t dst_w = src_w + left + right;
  uint32_t dst_h = src_h + top + bottom;
  int dst_y_size = dst_w * dst_h;
  size_t dst_size = dst_h * dst_w * 3 / 2;
  std::unique_ptr<char[]> unique(new char[dst_size]);
  auto dst = unique.get();
  char *dst_y = dst;
  char *dst_uv = dst + dst_y_size;
  auto *src_uv = src + src_h * src_w;
  uint32_t dst_bottom_start_line = src_h + top;
  // padding top and bottom
  for (uint32_t h = 0; h < dst_h; ++h) {  // padding y
    auto *raw = dst_y + h * dst_w + left;
    if (h < top) {  // padding top
      memcpy(raw, src, src_w);
    } else if (h >= dst_bottom_start_line) {  // padding bottom
      auto *src_y_raw = src + (src_h - 1) * src_w;
      memcpy(raw, src_y_raw, src_w);
    } else {  // copy src
      auto *src_y_raw = src + (h - top) * src_w;
      memcpy(raw, src_y_raw, src_w);
    }
  }

  for (uint32_t h = 0; h < dst_h / 2; ++h) {  // padding uv
    auto *raw = dst_uv + h * dst_w + left;
    if (h < (top / 2)) {  // top
      memcpy(raw, src_uv, src_w);
    } else if (h >= (dst_bottom_start_line / 2)) {  // bottom
      auto *src_uv_raw = src_uv + ((src_h / 2) - 1) * src_w;
      memcpy(raw, src_uv_raw, src_w);
    } else {  // copy src
      auto *src_uv_raw = src_uv + (h - (top / 2)) * src_w;
      memcpy(raw, src_uv_raw, src_w);
    }
  }

  // padding left and right
  for (uint32_t h = 0; h < dst_h; ++h) {
    // padding left
    auto *dst_left_y = dst_y + h * dst_w;
    auto *dst_left_uv = dst_uv + (h / 2) * dst_w;
    auto *src_left_y = dst_y + h * dst_w + left;
    auto *src_left_uv = dst_uv + (h / 2) * dst_w + left;
    uint16_t *src_left_uv_16 = (uint16_t *)(src_left_uv);
    memset(dst_left_y, *src_left_y, left);
    for (uint32_t w = 0; w < left; w += 2) {
      uint16_t *dst_16 = (uint16_t *)(dst_left_uv + w);
      *dst_16 = *src_left_uv_16;
    }
    // padding right
    auto *dst_right_y = dst_y + h * dst_w + left + src_w;
    auto *dst_right_uv = dst_uv + (h / 2) * dst_w + left + src_w;
    auto *src_right_y = dst_y + h * dst_w + left + src_w - 1;
    auto *src_right_uv = dst_uv + (h / 2) * dst_w + left + src_w - 2;

    uint16_t *src_right_uv_16 = (uint16_t *)(src_right_uv);
    memset(dst_right_y, *src_right_y, right);
    for (uint32_t w = 0; w < right; w += 2) {
      uint16_t *dst_16 = (uint16_t *)(dst_right_uv + w);
      *dst_16 = *src_right_uv_16;
    }
  }

  return unique;
}

std::unique_ptr<char[]> hobotcv_reflect_padding(const char *src,
                                                const int &src_h,
                                                const int &src_w,
                                                uint32_t top,
                                                uint32_t bottom,
                                                uint32_t left,
                                                uint32_t right) {
  uint32_t dst_w = src_w + left + right;
  uint32_t dst_h = src_h + top + bottom;
  int dst_y_size = dst_w * dst_h;
  size_t dst_size = dst_h * dst_w * 3 / 2;
  std::unique_ptr<char[]> unique(new char[dst_size]);
  auto dst = unique.get();
  char *dst_y = dst;
  char *dst_uv = dst + dst_y_size;
  auto *src_uv = src + src_h * src_w;
  uint32_t dst_bottom_start_line = src_h + top;
  // padding top and bottom
  int index_top = top, index_bottom = bottom;
  for (uint32_t h = 0; h < dst_h; ++h) {  // padding y
    auto *raw = dst_y + h * dst_w + left;
    if (h < top) {  // padding top
      auto *src_y_raw = src + index_top * src_w;
      memcpy(raw, src_y_raw, src_w);
      index_top--;
    } else if (h >= dst_bottom_start_line) {  // padding bottom
      auto *src_y_raw = src + (src_h - bottom + index_bottom - 1) * src_w;
      memcpy(raw, src_y_raw, src_w);
      index_bottom--;
    } else {  // copy src
      auto *src_y_raw = src + (h - top) * src_w;
      memcpy(raw, src_y_raw, src_w);
    }
  }

  index_top = top / 2;
  index_bottom = bottom / 2;
  for (uint32_t h = 0; h < dst_h / 2; ++h) {  // padding uv
    auto *raw = dst_uv + h * dst_w + left;
    if (h < (top / 2)) {  // top
      auto *src_uv_raw = src_uv + index_top * src_w;
      memcpy(raw, src_uv_raw, src_w);
      index_top--;
    } else if (h >= (dst_bottom_start_line / 2)) {  // bottom
      auto *src_uv_raw =
          src_uv + ((src_h / 2) - (bottom / 2) + index_bottom - 1) * src_w;
      memcpy(raw, src_uv_raw, src_w);
      index_bottom--;
    } else {  // copy src
      auto *src_uv_raw = src_uv + (h - (top / 2)) * src_w;
      memcpy(raw, src_uv_raw, src_w);
    }
  }

  // padding left and right
  for (uint32_t h = 0; h < dst_h; ++h) {
    // padding left
    auto *dst_left_y = dst_y + h * dst_w;
    auto *dst_left_uv = dst_uv + (h / 2) * dst_w;
    auto *src_left_y = dst_y + h * dst_w + left;
    auto *src_left_uv = dst_uv + (h / 2) * dst_w + left;
    for (uint32_t w = 0; w < left; w++) {
      *(dst_left_y + w) = *(src_left_y + left - w);
      if (w % 2 == 0) {
        uint16_t *dst_16 = (uint16_t *)(dst_left_uv + w);
        uint16_t *src_left_uv_16 = (uint16_t *)(src_left_uv + left - w);
        *dst_16 = *src_left_uv_16;
      }
    }
    // padding right
    auto *dst_right_y = dst_y + h * dst_w + left + src_w;
    auto *dst_right_uv = dst_uv + (h / 2) * dst_w + left + src_w;
    auto *src_right_y = dst_y + h * dst_w + left + src_w - 1;
    auto *src_right_uv = dst_uv + (h / 2) * dst_w + left + src_w - 2;
    for (uint32_t w = 0; w < right; w++) {
      *(dst_right_y + w) = *(src_right_y - w);
      if (w % 2 == 0) {
        uint16_t *dst_16 = (uint16_t *)(dst_right_uv + w);
        uint16_t *src_right_uv_16 = (uint16_t *)(src_right_uv - w);
        *dst_16 = *src_right_uv_16;
      }
    }
  }

  return unique;
}

hobotcv_front::hobotcv_front() {
  group_id = -1;
  roi.cropEnable = 0;
  roi.x = 0;
  roi.y = 0;
  roi.width = 0;
  roi.height = 0;
  pym_param.pymEnable = 0;
  observe = hobotcv_single::getSingleObj();
}

hobotcv_front::~hobotcv_front() {}

/*当加速方式为AUTO时，先验证vps条件，此时printLog为false。
若不适用vps加速，不输出error log，直接采用bpu加速方式。*/
int hobotcv_front::prepareResizeParam(int src_width,
                                      int src_height,
                                      int dst_width,
                                      int dst_height,
                                      bool printLog) {
  int resize_src_width = roi.cropEnable ? roi.width : src_width;
  int resize_src_height = roi.cropEnable ? roi.height : src_height;
  if (dst_width % 16 != 0) {
    int remain = dst_width % 16;
    int recommend_dst_width = dst_width + 16 - remain;
    if (printLog) {
      /*当加速方式为AUTO时，先验证vps条件，此时printLog为false。
      若不适用vps加速，不输出error log，直接采用bpu加速方式。*/
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv resize"),
                   "unsupported dst width %d! The dst width must "
                   "be a multiple of 16! The recommended dst width is %d ",
                   dst_width,
                   recommend_dst_width);
    }
    return -1;
  }
  if (dst_height % 2 != 0) {
    if (printLog) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv resize"),
                   "unsupported dst height %d! The dst height must be even!",
                   dst_height);
    }
    return -1;
  }
  if (dst_height > 2160 || dst_width > 4096 || dst_height < 32 ||
      dst_width < 32) {
    if (printLog) {
      RCLCPP_ERROR(
          rclcpp::get_logger("hobot_cv resize"),
          "unsupported dst resolution %d x %d! The supported dst resolution "
          "is 32 x 32 to 4096 x 2160!",
          dst_width,
          dst_height);
    }
    return -1;
  }
  if (resize_src_width % 16 != 0) {
    if (printLog) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv resize"),
                   "unsupported src width %d! The src width must "
                   "be a multiple of 16!",
                   resize_src_width);
    }
    return -1;
  }
  if (resize_src_height % 2 != 0) {
    if (printLog) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv resize"),
                   "unsupported src height %d! The src height must be even!",
                   resize_src_height);
    }
    return -1;
  }
  if (resize_src_height < 32 || resize_src_width < 32 ||
      resize_src_height > 2160 || resize_src_width > 4096) {
    if (printLog) {
      RCLCPP_ERROR(
          rclcpp::get_logger("hobot_cv resize"),
          "unsupported src resolution %d x %d! The supported src resolution is "
          "32 x 32 to 4096 x 2160",
          resize_src_width,
          resize_src_height);
    }
    return -1;
  }
  if (dst_height > resize_src_height * 1.5) {
    float height_ratio =
        static_cast<float>(dst_height) / static_cast<float>(resize_src_height);
    if (printLog) {
      RCLCPP_ERROR(
          rclcpp::get_logger("hobot_cv resize"),
          "Max 1.5x upscale is supported! Input dst height: %d src height: "
          "%d height ratio: %.2f>1.5x. Please change the src or dst height",
          dst_height,
          resize_src_height,
          height_ratio);
    }
    return -1;
  }
  if (dst_width > resize_src_width * 1.5) {
    float width_ratio =
        static_cast<float>(dst_width) / static_cast<float>(resize_src_width);
    if (printLog) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv resize"),
                   "Max 1.5x upscale is supported! dst width: %d src width: %d "
                   "width ratio: %.2f>1.5x. Please change the src or dst width",
                   dst_width,
                   resize_src_width,
                   width_ratio);
    }
    return -1;
  }
  if (dst_width < resize_src_width / 8) {
    float width_ratio =
        static_cast<float>(dst_width) / static_cast<float>(resize_src_width);
    if (printLog) {
      RCLCPP_ERROR(
          rclcpp::get_logger("hobot_cv resize"),
          "Max 1/8 downscale is supported! dst width: %d src width: "
          "%d width ratio: %.2f<1/8. Please change the src or dst width",
          dst_width,
          resize_src_width,
          width_ratio);
    }
    return -1;
  }
  if (dst_height < resize_src_height / 8) {
    float height_ratio =
        static_cast<float>(dst_height) / static_cast<float>(resize_src_height);
    if (printLog) {
      RCLCPP_ERROR(
          rclcpp::get_logger("hobot_cv resize"),
          "Max 1/8 downscale is supported! dst height: %d src height: "
          "%d height ratio: %.2f<1/8. Please change the src or dst height",
          dst_height,
          resize_src_height,
          height_ratio);
    }
    return -1;
  }
  this->src_h = src_height;
  this->src_w = src_width;
  this->dst_h = dst_height;
  this->dst_w = dst_width;
  return 0;
}

int hobotcv_front::prepareRotateParam(int width, int height, int rotation) {
  switch (rotation) {
    case 0:
      rotate = 0;
      return 1;
      break;
    case 1:
      rotate = 90;
      break;
    case 2:
      rotate = 180;
      break;
    case 3:
      rotate = 270;
      break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv rotate"),
                   "unsupport rotation: %d, hobot_cv only supports "
                   "ROTATION_90、ROTATION_180、ROTATION_270!",
                   rotation);
      return -1;
      break;
  }
  if (rotate == 90 || rotate == 270) {
    if (width % 16 != 0 || height % 16 != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv rotate"),
                   "unsupported src resolution %d x %d, the width and height "
                   "must be a multiple of 16!",
                   width,
                   height);
      return -1;
    }
  } else {
    if (width % 16 != 0) {
      RCLCPP_ERROR(
          rclcpp::get_logger("hobot_cv rotate"),
          "unsupported src width: %d, the width must be a multiple of 16!",
          width);
      return -1;
    }
    if (height % 2 != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv rotate"),
                   "unsupported src height %d! The src height must be even!",
                   height);
    }
  }

  if (width > 4096 || height > 2160 || width < 32 || height < 32) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv rotate"),
                 "unsupported src resolution %d x %d, The supported src "
                 "resolution is 32 x "
                 "32 to 4096 x 2160",
                 width,
                 height);
    return -1;
  }
  return 0;
}

/*当加速方式为AUTO时，先验证vps条件，此时printLog为false。
若不适用vps加速，不输出error log，直接采用bpu加速方式。*/
int hobotcv_front::prepareCropRoi(int src_height,
                                  int src_width,
                                  int dst_width,
                                  int dst_height,
                                  const cv::Range &rowRange,
                                  const cv::Range &colRange,
                                  bool printLog) {
  if (colRange.end - colRange.start <= 0 ||
      rowRange.end - rowRange.start <= 0) {
    roi.cropEnable = 0;
  } else {
    if (rowRange.start < 0 || colRange.start < 0 || rowRange.end > src_height ||
        colRange.end > src_width) {
      if (printLog) {
        RCLCPP_ERROR(
            rclcpp::get_logger("hobot_cv crop"),
            "Invalid Range data, rowRange.start:%d rowRange.end:%d "
            "colRange.start: %d colRange.end: %d"
            "rowRange should be in [0, %d) and colRange should be in [0, %d)",
            rowRange.start,
            rowRange.end,
            colRange.start,
            colRange.end,
            src_height,
            src_width);
      }
      return -1;
    }

    if (dst_height > (rowRange.end - rowRange.start) * 1.5) {
      int crop_height = rowRange.end - rowRange.start;
      float height_ratio =
          static_cast<float>(dst_height) / static_cast<float>(crop_height);
      if (printLog) {
        RCLCPP_ERROR(
            rclcpp::get_logger("hobot_cv crop"),
            "Max 1.5x upscale is supported! dst height: %d cropArea height: %d "
            "height ratio: %.2f>1.5x. Please change the dst or cropArea height",
            dst_height,
            crop_height,
            height_ratio);
      }
      return -1;
    }

    if (dst_width > (colRange.end - colRange.start) * 1.5) {
      int crop_width = colRange.end - colRange.start;
      float width_ratio =
          static_cast<float>(dst_width) / static_cast<float>(crop_width);
      if (printLog) {
        RCLCPP_ERROR(
            rclcpp::get_logger("hobot_cv crop"),
            "Max 1.5x upscale is supported! dst width: %d cropArea width: %d "
            "width ratio: %.2f>1.5x. Please change the src or cropArea width",
            dst_width,
            crop_width,
            width_ratio);
      }
      return -1;
    }
    if (dst_width < (colRange.end - colRange.start) / 8) {
      int crop_width = colRange.end - colRange.start;
      float width_ratio =
          static_cast<float>(dst_width) / static_cast<float>(crop_width);
      if (printLog) {
        RCLCPP_ERROR(
            rclcpp::get_logger("hobot_cv crop"),
            "Max 1/8 downscale is supported! dst width: %d cropArea width: %d "
            "width ratio: %.2f<1/8. Please change the src or cropArea width",
            dst_width,
            crop_width,
            width_ratio);
      }
      return -1;
    }
    if (dst_height < (rowRange.end - rowRange.start) / 8) {
      int crop_height = rowRange.end - rowRange.start;
      float height_ratio =
          static_cast<float>(dst_height) / static_cast<float>(crop_height);
      if (printLog) {
        RCLCPP_ERROR(
            rclcpp::get_logger("hobot_cv crop"),
            "Max 1/8 downscale is supported! dst height: %d cropArea "
            "height: %d height ratio: %.2f<1/8. Please change the src or "
            "cropArea height",
            dst_height,
            crop_height,
            height_ratio);
      }
      return -1;
    }
    roi.cropEnable = 1;
    roi.x = colRange.start;
    roi.y = rowRange.start;
    roi.width = colRange.end - colRange.start;
    roi.height = rowRange.end - rowRange.start;
  }
  return 0;
}

int hobotcv_front::preparePymraid(int src_h,
                                  int src_w,
                                  const PyramidAttr &attr) {
  if (src_h > 2160 || src_w > 4096 || src_h < 64 || src_w < 64) {
    RCLCPP_ERROR(
        rclcpp::get_logger("hobot_cv pyramid"),
        "unsupported src resolution %d x %d! The supported src resolution is "
        "64 x 64 to 4096 x 4096!",
        src_w,
        src_h);
    return -1;
  }
  if (attr.ds_info[0].factor == 0 || attr.ds_info[4].factor == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv pyramid"),
                 "ds_info[0].factor: %d ds_info[4].factor: %d, base0 and base "
                 "4 must enable!",
                 attr.ds_info[0].factor,
                 attr.ds_info[4].factor);
    return -1;
  }
  memcpy(&pym_param.attr, &attr, sizeof(PyramidAttr));
  pym_param.pymEnable = 1;
  this->src_h = src_h;
  this->src_w = src_w;
  for (size_t i = 0; i < 24; i++) {
    if (pym_param.attr.ds_info[i].factor != 0) {
      ds_layer_en = i;
    }
  }
  return 0;
}

int hobotcv_front::groupScheduler() {
  processId = getpid();
  //选取crop区域的宽高作为输入源宽高
  src_h = roi.cropEnable == 1 ? roi.height : src_h;
  src_w = roi.cropEnable == 1 ? roi.width : src_w;

  static int sem_groups_times = 0;  //记录信号量超时次数
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 2;  // 超时2秒
  int sem_groups_ret = sem_timedwait(observe->fifo.sem_groups, &ts);
  if (sem_groups_ret != 0) {
    sem_groups_times++;
    if (sem_groups_times >= 3) {  //连续三次未获取到信号量，重置hobot_cv
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                   "Group abnormal block, reset hobot_cv!");
      sem_groups_times = 0;
      //关闭group
      for (int i = 0; i < HOBOTCV_GROUP_SIZE; i++) {
        Group_info_t *group = (Group_info_t *)(observe->fifo.groups) + i;
        if (group->group_id >= HOBOTCV_GROUP_BEGIN) {
          HB_VPS_StopGrp(group->group_id);
          HB_VPS_DestroyGrp(group->group_id);
        }
      }
      //共享内存重新初始化
      size_t size = sizeof(Group_info_t) * HOBOTCV_GROUP_SIZE;
      memset(observe->fifo.groups, 0, size);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "wait sem_groups time out!");
      return -1;
    }
  } else {
    sem_groups_times = 0;
  }

  bool have_same_group = false;
  for (int i = 0; i < HOBOTCV_GROUP_SIZE; i++) {
    Group_info_t *group = (Group_info_t *)(observe->fifo.groups) + i;
    if (group->group_state == 1) {  //已经被系统释放
      continue;
    }
    if (group->process_id == processId) {
      if (group->max_h == src_h && src_w == group->max_w) {
        if (pym_param.pymEnable == 1) {  //本次操作为pyramid
          int pym_channel = 2;
          if (src_w > 2048 || src_h > 1080) {
            // pym输入分辨率大于2048x1080，pym选择通道2
            pym_channel = 2;
          } else {  // pym输入分辨率小于2048x1080，channel 1 即可满足需求
            pym_channel = 1;
          }
          if (group->channels[2].pym_enable == 1) {  //通道2已经被设置为pym通道
            have_same_group = true;
          } else if (group->channels[1].pym_enable ==
                     1) {  //通道1已经被设置为pym通道
            if (pym_channel == 1) {
              have_same_group = true;
            }
          }
        } else {
          if (dst_w > src_w || dst_h > src_h) {
            // up scale 默认使用通道5
            have_same_group = true;
          } else {
            // down scale 使用通道2
            if (group->channels[2].pym_enable == 0) {
              //通道2未被设置为pym通道，可用
              have_same_group = true;
            }
          }
        }
        if (have_same_group) {
          group_id = i + HOBOTCV_GROUP_BEGIN;
          group->active_time = currentMicroseconds();
          break;
        }
      }
    }
  }
  if (!have_same_group) {
    auto ret = createGroup();
    if (ret != 0) {
      sem_post(observe->fifo.sem_groups);
      return -1;
    }
  }
  auto ret = group_sem_wait();
  if (ret != 0) {
    sem_post(observe->fifo.sem_groups);
    return -1;
  }

  setVpsChannelAttr();  //设置vps通道属性

  ret = HB_VPS_EnableChn(group_id, channel_id);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "%d EnableChn: %d failed ret: %d!!",
                 group_id,
                 channel_id,
                 ret);
    group_sem_post();
    sem_post(observe->fifo.sem_groups);
    return -1;
  }
  sem_post(observe->fifo.sem_groups);
  return 0;
}

int hobotcv_front::setVpsChannelAttr() {
  int enScale = 1;
  if (pym_param.pymEnable == 1) {  //本次操作为pym操作，设置pym通道输出大小
    setChannelPyramidAttr();
  } else {
    if (dst_w > src_w || dst_h > src_h) {  // up scale
      channel_id = 5;
    } else {  // down scale
      channel_id = 2;
    }
    if (dst_w == src_w || dst_h == src_h) {
      enScale = 0;
    }
    int group_index = group_id - HOBOTCV_GROUP_BEGIN;
    Group_info_t *group = (Group_info_t *)(observe->fifo.groups) + group_index;
    if (group->channels[channel_id].output_w != dst_w ||
        group->channels[channel_id].output_h != dst_h) {
      setChannelAttr(enScale);
      group->channels[channel_id].output_w = dst_w;
      group->channels[channel_id].output_h = dst_h;
    }

    group->channels[channel_id].rotation = rotate;
    setChannelRotate();
  }
  return 0;
}

int hobotcv_front::createGroup() {
  auto now_time = currentMicroseconds();
  for (int i = 0; i < HOBOTCV_GROUP_SIZE; i++) {
    Group_info_t *group = (Group_info_t *)(observe->fifo.groups) + i;
    int groupId = i + HOBOTCV_GROUP_BEGIN;
    if (group->active_time == 0) {
      group->group_id = groupId;
      group->max_h = src_h;
      group->max_w = src_w;
      group->active_time = currentMicroseconds();
      group->process_id = processId;
      group->group_state = 0;
      group_id = groupId;
      break;
    } else if (group->group_state == 1) {  //被系统回收
      group->group_id = groupId;
      group->max_h = src_h;
      group->max_w = src_w;
      group->active_time = currentMicroseconds();
      group->process_id = processId;
      group->group_state = 0;
      group_id = groupId;
      break;
    } else if (now_time - group->active_time > HOBOTCV_GROUP_OVER_TIME) {
      //超时，hobotcv回收
      HB_VPS_StopGrp(group->group_id);
      HB_VPS_DestroyGrp(group->group_id);
      memset(group->channels, 0, sizeof(Channel_info_t) * 7);
      group->group_id = groupId;
      group->max_h = src_h;
      group->max_w = src_w;
      group->active_time = currentMicroseconds();
      group->process_id = processId;
      group->group_state = 0;
      memset(group->channels, 0, sizeof(Channel_info_t) * 7);
      group_id = groupId;
      break;
    }
  }

  if (-1 == group_id) {
    //未找到空闲group
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "hobot_cv group is full !!");
    return -1;
  }

  VPS_GRP_ATTR_S grp_attr;
  grp_attr.maxW = src_w;
  grp_attr.maxH = src_h;
  grp_attr.frameDepth = 1;
  auto ret = HB_VPS_CreateGrp(group_id, &grp_attr);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "create group: %d failed! ret: %d",
                 group_id,
                 ret);
    return -1;
  }

  hobotcv_sys_mem sys_mem;
  //申请系统内存
  int alloclen = src_h * src_w;
  ret = HB_SYS_Alloc(
      &(sys_mem.mmz_paddr[0]), (void **)&(sys_mem.mmz_vaddr[0]), alloclen);
  if (0 != ret) {
    RCLCPP_ERROR(
        rclcpp::get_logger("hobot_cv"), "HB_SYS_Alloc failed ret: %d!!", ret);
    return -1;
  }
  alloclen = src_h * src_w / 2;
  HB_SYS_Alloc(
      &(sys_mem.mmz_paddr[1]), (void **)&(sys_mem.mmz_vaddr[1]), alloclen);
  if (ret != 0) {
    RCLCPP_ERROR(
        rclcpp::get_logger("hobot_cv"), "HB_SYS_Alloc failed! ret: %d", ret);
    return ret;
  }
  std::unique_lock<std::mutex> lk(observe->group_map_mtx);
  observe->HobotcvAddGroup(group_id, sys_mem);
  lk.unlock();
  //启用channel 1，2，5
  groupChn1Init(group_id, grp_attr.maxW, grp_attr.maxH);
  groupChn2Init(group_id, grp_attr.maxW, grp_attr.maxH);
  groupChn5Init(group_id, grp_attr.maxW, grp_attr.maxH);
  if (pym_param.pymEnable == 1) {
    groupPymChnInit(group_id, grp_attr.maxW, grp_attr.maxH);
  }

  ret = HB_VPS_StartGrp(group_id);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "StartGrp: %d failed! ret: %d",
                 group_id,
                 ret);
    return -1;
  }
  return 0;
}

int hobotcv_front::setChannelAttr(int enscale) {
  VPS_CHN_ATTR_S chn_attr;
  memset(&chn_attr, 0, sizeof(chn_attr));
  chn_attr.width = dst_w;
  chn_attr.height = dst_h;
  chn_attr.enScale = enscale;
  chn_attr.frameDepth = 1;
  int ret = HB_VPS_SetChnAttr(group_id, channel_id, &chn_attr);
  if (0 != ret) {
    RCLCPP_ERROR(
        rclcpp::get_logger("hobot_cv"), "SetChnAttr failed! ret: %d", ret);
    return ret;
  }
  return 0;
}

int hobotcv_front::setChannelRotate() {
  HB_ROTATION_E hb_rotate;
  if (rotate == 0) {
    hb_rotate = HB_ROTATION_E::ROTATION_0;
  } else if (rotate == 90) {
    hb_rotate = HB_ROTATION_E::ROTATION_90;
  } else if (rotate == 180) {
    hb_rotate = HB_ROTATION_E::ROTATION_180;
  } else if (rotate == 270) {
    hb_rotate = HB_ROTATION_E::ROTATION_270;
  }
  int ret = HB_VPS_SetChnRotate(group_id, channel_id, hb_rotate);
  if (0 != ret) {
    RCLCPP_ERROR(
        rclcpp::get_logger("hobot_cv"), "SetChnRotate failed! ret: %d", ret);
    return ret;
  }
  return 0;
}

int hobotcv_front::setChannelPyramidAttr() {
  if (src_w > 2048 || src_h > 1080) {
    // pym输入分辨率大于2048x1080，需要将channel2设置为pym通道
    channel_id = 2;
  } else {  // pym输入分辨率小于2048x1080，channel 1 即可满足需求
    channel_id = 1;
  }
  VPS_PYM_CHN_ATTR_S pym_chn_attr;
  memset(&pym_chn_attr, 0, sizeof(pym_chn_attr));
  pym_chn_attr.timeout = pym_param.attr.timeout;
  pym_chn_attr.ds_layer_en = ds_layer_en;
  pym_chn_attr.frame_id = 0;
  pym_chn_attr.frameDepth = 1;
  memcpy(pym_chn_attr.ds_info,
         pym_param.attr.ds_info,
         sizeof(pym_chn_attr.ds_info));

  auto ret = HB_VPS_SetPymChnAttr(group_id, channel_id, &pym_chn_attr);
  if (ret != 0) {
    RCLCPP_ERROR(
        rclcpp::get_logger("hobot_cv"), "set pym chn failed!ret: %d", ret);
    return -1;
  }
  return 0;
}

int hobotcv_front::sendVpsFrame(const cv::Mat &src) {
  int input_w = src.cols;
  int input_h = src.rows * 2 / 3;
  std::unique_lock<std::mutex> lk(observe->group_map_mtx);
  if (roi.cropEnable == 1) {  // crop区域作为vps输入源
    auto srcdata = reinterpret_cast<const uint8_t *>(src.data);
    // copy y
    for (int h = 0; h < roi.height; ++h) {
      auto *raw =
          observe->GetGroupSysmem(group_id).mmz_vaddr[0] + h * roi.width;
      auto *src = srcdata + (h + roi.y) * input_w + roi.x;
      memcpy(raw, src, roi.width);
    }
    // copy uv
    auto uv_data = srcdata + input_h * input_w;
    for (int32_t h = 0; h < roi.height / 2; ++h) {
      auto *raw =
          observe->GetGroupSysmem(group_id).mmz_vaddr[1] + h * roi.width;
      auto *src = uv_data + (h + (roi.y / 2)) * input_w + roi.x;
      memcpy(raw, src, roi.width);
    }
  } else {
    auto ydata = reinterpret_cast<const uint8_t *>(src.data);
    auto uvdata = ydata + src_h * src_w;
    memcpy(
        observe->GetGroupSysmem(group_id).mmz_vaddr[0], ydata, src_h * src_w);
    memcpy(observe->GetGroupSysmem(group_id).mmz_vaddr[1],
           uvdata,
           src_h * src_w / 2);
  }
  hb_vio_buffer_t feedback_buf;
  feedback_buf.img_addr.width = src_w;
  feedback_buf.img_addr.height = src_h;
  feedback_buf.img_addr.stride_size = src_w;

  feedback_buf.img_addr.addr[0] =
      observe->GetGroupSysmem(group_id).mmz_vaddr[0];
  feedback_buf.img_addr.addr[1] =
      observe->GetGroupSysmem(group_id).mmz_vaddr[1];
  feedback_buf.img_addr.paddr[0] =
      observe->GetGroupSysmem(group_id).mmz_paddr[0];
  feedback_buf.img_addr.paddr[1] =
      observe->GetGroupSysmem(group_id).mmz_paddr[1];
  auto ret = HB_VPS_SendFrame(group_id, &feedback_buf, 1000);
  lk.unlock();
  if (0 != ret) {
    RCLCPP_ERROR(
        rclcpp::get_logger("hobot_cv"), "SendFrame failed! ret: %d", ret);
    return ret;
  }
  return 0;
}

int hobotcv_front::getChnFrame(cv::Mat &dst) {
  hb_vio_buffer_t out_buf;
  int ret = HB_VPS_GetChnFrame(group_id, channel_id, &out_buf, 2000);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "get group: %d chn: %d frame failed! ret: %d",
                 group_id,
                 channel_id,
                 ret);
    group_sem_post();
    return -1;
  }

  int stride = out_buf.img_addr.stride_size;
  int width = out_buf.img_addr.width;
  int height = out_buf.img_addr.height;

  dst = cv::Mat(height * 3 / 2, width, CV_8UC1);
  copyOutputImage(stride,
                  width,
                  height,
                  out_buf.img_addr,
                  reinterpret_cast<char *>(dst.data));
  HB_VPS_ReleaseChnFrame(group_id, channel_id, &out_buf);
  HB_VPS_DisableChn(group_id, channel_id);
  group_sem_post();
  return 0;
}

int hobotcv_front::getPyramidOutputImage(OutputPyramid *pymOut) {
  pym_buffer_t pym_out;
  auto ret = HB_VPS_GetChnFrame(
      group_id, channel_id, (void *)&pym_out, pym_param.attr.timeout);
  if (ret == 0) {
    pymOut->isSuccess = true;
    int ds_base_index = -1;
    for (int i = 0; i < 24; i++) {
      if (i % 4 == 0) {  // ds 基础层
        ds_base_index++;
        int out_w = pym_out.pym[ds_base_index].width;
        int out_h = pym_out.pym[ds_base_index].height;
        int stride = pym_out.pym[ds_base_index].stride_size;
        if (out_w != 0 && out_h != 0 && pym_param.attr.ds_info[i].factor != 0) {
          pymOut->pym_out[i].width = out_w;
          pymOut->pym_out[i].height = out_h;
          int size = out_w * out_h * 3 / 2;
          pymOut->pym_out[i].img.resize(size);
          copyOutputImage(stride,
                          out_w,
                          out_h,
                          pym_out.pym[ds_base_index],
                          &(pymOut->pym_out[i].img[0]));
        } else {
          pymOut->pym_out[i].width = 0;
          pymOut->pym_out[i].height = 0;
        }
      } else {  // ds roi层
        int roi_index = i - ds_base_index * 4 - 1;
        if (pym_param.attr.ds_info[i].factor != 0) {
          int out_w = pym_out.pym_roi[ds_base_index][roi_index].width;
          int out_h = pym_out.pym_roi[ds_base_index][roi_index].height;
          int stride = pym_out.pym_roi[ds_base_index][roi_index].stride_size;
          pymOut->pym_out[i].width = out_w;
          pymOut->pym_out[i].height = out_h;
          int size = out_w * out_h * 3 / 2;
          pymOut->pym_out[i].img.resize(size);
          copyOutputImage(stride,
                          out_w,
                          out_h,
                          pym_out.pym_roi[ds_base_index][roi_index],
                          &(pymOut->pym_out[i].img[0]));
        } else {
          pymOut->pym_out[i].width = 0;
          pymOut->pym_out[i].height = 0;
        }
      }
    }
    HB_VPS_ReleaseChnFrame(group_id, channel_id, &pym_out);
    HB_VPS_DisableChn(group_id, channel_id);
  } else {
    pymOut->isSuccess = false;
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "group: %d chn: %d get frame failed! ret: %d",
                 group_id,
                 channel_id,
                 ret);
  }

  group_sem_post();
  return 0;
}

int hobotcv_front::group_sem_wait() {
  std::chrono::milliseconds timeout(2000);  //超时2s

  bool ret = false;
  if (4 == group_id) {
    ret = observe->fifo.mtx_group4.try_lock_for(timeout);
  } else if (5 == group_id) {
    ret = observe->fifo.mtx_group5.try_lock_for(timeout);
  } else if (6 == group_id) {
    ret = observe->fifo.mtx_group6.try_lock_for(timeout);
  } else if (7 == group_id) {
    ret = observe->fifo.mtx_group7.try_lock_for(timeout);
  } else {
    return -1;
  }
  if (!ret) {  //等待超时
    RCLCPP_ERROR(
        rclcpp::get_logger("hobot_cv"), "wait group: %d time out ", group_id);
    std::unique_lock<std::mutex> lk(observe->group_map_mtx);
    observe->AddGroupTimeOut(group_id);
    if (observe->GetGroupTimeOut(group_id) >= 3) {  //超时三次，重置group
      observe->ResetGroupTimeOutNum(group_id);
      Group_info_t *group = (Group_info_t *)(observe->fifo.groups) +
                            (group_id - HOBOTCV_GROUP_BEGIN);
      //超时，hobotcv回收
      HB_VPS_StopGrp(group_id);
      HB_VPS_DestroyGrp(group_id);
      memset(group, 0, sizeof(Group_info_t));
      group_sem_post();
    }
    lk.unlock();
    return -1;
  } else {
    observe->ResetGroupTimeOutNum(group_id);
  }
  return 0;
}

int hobotcv_front::group_sem_post() {
  if (4 == group_id) {
    observe->fifo.mtx_group4.unlock();
  } else if (5 == group_id) {
    observe->fifo.mtx_group5.unlock();
  } else if (6 == group_id) {
    observe->fifo.mtx_group6.unlock();
  } else if (7 == group_id) {
    observe->fifo.mtx_group7.unlock();
  } else {
    return -1;
  }
  return 0;
}

int hobotcv_front::groupChn1Init(int group_id, int max_w, int max_h) {
  VPS_CHN_ATTR_S chn_attr_max;
  memset(&chn_attr_max, 0, sizeof(chn_attr_max));
  chn_attr_max.width = max_w > 2048 ? 2048 : max_w;
  chn_attr_max.height = max_h > 1080 ? 1080 : max_h;
  chn_attr_max.enScale = 1;
  chn_attr_max.frameDepth = 1;
  auto ret = HB_VPS_SetChnAttr(group_id, 1, &chn_attr_max);
  if (ret != 0) {
    RCLCPP_ERROR(
        rclcpp::get_logger("hobot_cv"),
        "group: %d Chn1Init failed! ret: %d chn_width: %d chn_height: %d",
        group_id,
        ret,
        chn_attr_max.width,
        chn_attr_max.height);
  }
  return 0;
}

int hobotcv_front::groupChn2Init(int group_id, int max_w, int max_h) {
  VPS_CHN_ATTR_S chn_attr_max;
  memset(&chn_attr_max, 0, sizeof(chn_attr_max));
  chn_attr_max.width = max_w > 4096 ? 4096 : max_w;
  chn_attr_max.height = max_h > 2156 ? 2156 : max_h;
  chn_attr_max.enScale = 1;
  chn_attr_max.frameDepth = 1;
  auto ret = HB_VPS_SetChnAttr(group_id, 2, &chn_attr_max);
  if (ret != 0) {
    RCLCPP_ERROR(
        rclcpp::get_logger("hobot_cv"),
        "group: %d Chn2Init failed! ret: %d chn_width: %d chn_height: %d",
        group_id,
        ret,
        chn_attr_max.width,
        chn_attr_max.height);
  }
  return 0;
}

int hobotcv_front::groupChn5Init(int group_id, int max_w, int max_h) {
  VPS_CHN_ATTR_S chn_attr_max;
  memset(&chn_attr_max, 0, sizeof(chn_attr_max));
  int max_us_w = max_w * 1.5;
  int max_us_h = max_h * 1.5;
  max_us_w = max_us_w - (max_us_w % 16);
  chn_attr_max.width = max_us_w > 4096 ? 4096 : max_us_w;
  chn_attr_max.height = max_us_h > 2160 ? 2160 : max_us_h;
  chn_attr_max.enScale = 1;
  chn_attr_max.frameDepth = 1;
  auto ret = HB_VPS_SetChnAttr(group_id, 5, &chn_attr_max);
  if (ret != 0) {
    RCLCPP_ERROR(
        rclcpp::get_logger("hobot_cv"),
        "group: %d Chn5Init failed! ret: %d chn_width: %d chn_height: %d",
        group_id,
        ret,
        chn_attr_max.width,
        chn_attr_max.height);
  }
  return 0;
}

int hobotcv_front::groupPymChnInit(int group_id, int max_w, int max_h) {
  int pym_channel = 2;
  if (max_w > 2048 || max_h > 1080) {
    pym_channel = 2;
  } else {
    pym_channel = 1;
  }

  VPS_PYM_CHN_ATTR_S pym_chn_attr;
  memset(&pym_chn_attr, 0, sizeof(pym_chn_attr));
  pym_chn_attr.timeout = 2000;
  pym_chn_attr.ds_layer_en = 23;
  pym_chn_attr.frame_id = 0;
  pym_chn_attr.frameDepth = 1;
  int ds_base_index = -1;
  int roi_maxW = max_w;
  int roi_maxH = max_h;
  for (size_t i = 0; i < 24; i++) {  //初始化roi层
    if (i % 4 == 0) {
      ds_base_index++;
      if (ds_base_index == 0) {
        roi_maxW = max_w;
        roi_maxH = max_h;
      } else {
        roi_maxW = roi_maxW / 2;
        roi_maxH = roi_maxH / 2;
        roi_maxW = (roi_maxW % 2) == 0 ? roi_maxW : (roi_maxW - 1);
        roi_maxH = (roi_maxH % 2) == 0 ? roi_maxH : (roi_maxH - 1);
      }
    }
    if (i % 4 != 0) {
      int roi_tag_x = (roi_maxW - 1) * 64 / (64 + 1) + 1;
      int roi_tag_y = (((roi_maxH / 2 - 1) * 64 / (64 + 1)) + 1) * 2;
      if (roi_tag_x < 48 || roi_tag_y < 32) {
        pym_chn_attr.ds_info[i].factor = 0;
      } else {
        pym_chn_attr.ds_info[i].factor = 1;
      }

      pym_chn_attr.ds_info[i].roi_width = roi_maxW;
      pym_chn_attr.ds_info[i].roi_height = roi_maxH;
    }
  }

  auto ret = HB_VPS_SetPymChnAttr(group_id, pym_channel, &pym_chn_attr);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "group: %d pymChnInit failed! ret: %d",
                 group_id,
                 ret);
    return -1;
  }
  int group_index = group_id - HOBOTCV_GROUP_BEGIN;
  Group_info_t *group = (Group_info_t *)(observe->fifo.groups) + group_index;
  group->channels[pym_channel].pym_enable = 1;
  return 0;
}

int hobotcv_front::copyOutputImage(
    int stride, int width, int height, address_info_t &img_addr, void *output) {
  if (stride == width) {
    memcpy(output, img_addr.addr[0], width * height);
    memcpy(
        (char *)output + width * height, img_addr.addr[1], width * height / 2);
  } else {
    int i = 0;
    // jump over stride - width Y
    for (i = 0; i < height; i++) {
      memcpy((char *)output + i * width, img_addr.addr[0] + i * stride, width);
    }
    // jump over stride - width UV
    for (i = 0; i < height / 2; i++) {
      memcpy((char *)output + width * height + i * width,
             img_addr.addr[1] + i * stride,
             width);
    }
  }
  return 0;
}

}  // namespace hobot_cv
