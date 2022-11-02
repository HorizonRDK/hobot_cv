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

#include "hobotcv_imgproc/hobotcv_imgproc.h"

#include <iostream>

#include "hobotcv_imgproc/hobotcv_front.h"
#include "include/utils.h"
#include "rclcpp/rclcpp.hpp"

namespace hobot_cv {
int hobotcv_vps_resize(const cv::Mat &src,
                       cv::Mat &dst,
                       int dst_h,
                       int dst_w,
                       const cv::Range &rowRange,
                       const cv::Range &colRange) {
  int src_h = src.rows * 2 / 3;
  int src_w = src.cols;

  hobotcv_front hobotcv;
  int ret =
      hobotcv.prepareCropRoi(src_h, src_w, dst_w, dst_h, rowRange, colRange);
  if (ret != 0) {
    return -1;
  }

  ret = hobotcv.prepareResizeParam(src_w, src_h, dst_w, dst_h);
  if (0 != ret) {
    return -1;
  }

  if (hobotcv.roi.height == dst_h && hobotcv.roi.width == dst_w) {  // only crop
    auto srcdata = reinterpret_cast<const uint8_t *>(src.data);
    dst = cv::Mat(dst_h * 3 / 2, dst_w, CV_8UC1);
    auto dstdata = dst.data;
    // copy y
    for (int h = 0; h < dst_h; ++h) {
      auto *raw = dstdata + h * dst_w;
      auto *src = srcdata + (h + hobotcv.roi.y) * src_w + hobotcv.roi.x;
      memcpy(raw, src, dst_w);
    }

    // copy uv
    auto uv_data = srcdata + src_h * src_w;
    auto dstuvdata = dstdata + dst_h * dst_w;
    for (int32_t h = 0; h < dst_h / 2; ++h) {
      auto *raw = dstuvdata + h * dst_w;
      auto *src = uv_data + (h + (hobotcv.roi.y / 2)) * src_w + hobotcv.roi.x;
      memcpy(raw, src, dst_w);
    }
    return 0;
  }

  ret = hobotcv.groupScheduler();
  if (ret != 0) {
    return -1;
  }
  ret = hobotcv.sendVpsFrame(src);
  if (ret != 0) {
    return -1;
  }
  ret = hobotcv.getChnFrame(dst);
  if (ret != 0) {
    return -1;
  }
  return 0;
}

int hobotcv_resize(const cv::Mat &src,
                   int src_h,
                   int src_w,
                   cv::Mat &dst,
                   int dst_h,
                   int dst_w,
                   HobotcvSpeedUpType type) {
  if (type == HOBOTCV_AUTO) {
    hobotcv_front hobotcv;
    auto ret = hobotcv.prepareResizeParam(src_w, src_h, dst_w, dst_h, false);
    if (ret == 0) {
      return hobotcv_vps_resize(
          src, dst, dst_h, dst_w, cv::Range(0, 0), cv::Range(0, 0));
    }
  } else if (type == HOBOTCV_VPS) {
    return hobotcv_vps_resize(
        src, dst, dst_h, dst_w, cv::Range(0, 0), cv::Range(0, 0));
  }
  auto ret = prepareBpuResizeParam(src_w, src_h, dst_w, dst_h);
  if (ret != 0) {
    return ret;
  }
  hbDNNTensor input_tensor;
  prepare_nv12_tensor_without_padding(src.data, src_h, src_w, &input_tensor);
  // Prepare output tensor
  hbDNNTensor output_tensor;
  prepare_nv12_tensor_without_padding(dst_h, dst_w, &output_tensor);
  // resize
  hbDNNResizeCtrlParam ctrl = {
      HB_BPU_CORE_0, 0, HB_DNN_RESIZE_TYPE_BILINEAR, 0, 0, 0, 0};
  hbDNNTaskHandle_t task_handle = nullptr;

  ret =
      hbDNNResize(&task_handle, &output_tensor, &input_tensor, nullptr, &ctrl);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "hbDNNResize failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return ret;
  }
  ret = hbDNNWaitTaskDone(task_handle, 0);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "hbDNNWaitTaskDone failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return ret;
  }
  ret = hbDNNReleaseTask(task_handle);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "release task  failed!!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return ret;
  }

  size_t size = dst_h * dst_w * 3 / 2;
  dst = cv::Mat(dst_h * 3 / 2, dst_w, CV_8UC1);
  memcpy(dst.data, output_tensor.sysMem[0].virAddr, size);
  ret = hbSysFreeMem(&(input_tensor.sysMem[0]));
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "Free input_tensor mem failed!!");
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return ret;
  }
  ret = hbSysFreeMem(&(output_tensor.sysMem[0]));
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "Free output_tensor mem failed!!");
    return ret;
  }
  return 0;
}

cv::Mat hobotcv_crop(const cv::Mat &src,
                     int src_h,
                     int src_w,
                     int dst_h,
                     int dst_w,
                     const cv::Range &rowRange,
                     const cv::Range &colRange,
                     HobotcvSpeedUpType type) {
  if (type == HOBOTCV_AUTO) {
    hobotcv_front hobotcv;
    auto ret = hobotcv.prepareCropRoi(
        src_h, src_w, dst_w, dst_h, rowRange, colRange, false);
    if (ret == 0) {
      ret = hobotcv.prepareResizeParam(src_w, src_h, dst_w, dst_h, false);
      if (ret == 0) {
        cv::Mat dst;
        hobotcv_vps_resize(src, dst, dst_h, dst_w, rowRange, colRange);
        return dst;
      }
    }
  }
  if (type == HOBOTCV_VPS) {
    cv::Mat dst;
    hobotcv_vps_resize(src, dst, dst_h, dst_w, rowRange, colRange);
    return dst;
  }
  cv::Mat dst(dst_h * 3 / 2, dst_w, CV_8UC1);
  if (rowRange.end > src_h || colRange.end > src_w || rowRange.start < 0 ||
      colRange.start < 0) {  // crop区域要在原图范围内
    RCLCPP_ERROR(
        rclcpp::get_logger("hobot_cv crop"),
        "Invalid Range data, rowRange.start:%d rowRange.end:%d "
        "colRange.start: %d colRange.end: %d"
        "rowRange should be in [0, %d) and colRange should be in [0, %d)",
        rowRange.start,
        rowRange.end,
        colRange.start,
        colRange.end,
        src_h,
        src_w);
    return dst;
  }

  hbDNNRoi roi;
  int range_h = 0, range_w = 0;
  if (colRange.end - colRange.start <= 0 ||
      rowRange.end - rowRange.start <= 0) {
    roi.left = 0;
    roi.top = 0;
    roi.right = 0;
    roi.bottom = 0;
  } else {
    roi.left = colRange.start;
    roi.top = rowRange.start;
    roi.right = (colRange.end - 1) < 0 ? 0 : (colRange.end - 1);
    roi.bottom = (rowRange.end - 1) < 0 ? 0 : (rowRange.end - 1);
    range_h = rowRange.end - rowRange.start;
    range_w = colRange.end - colRange.start;
  }

  if (range_h == dst_h && range_w == dst_w) {
    auto srcdata = reinterpret_cast<const uint8_t *>(src.data);
    auto dstdata = dst.data;
    // copy y
    for (int h = 0; h < dst_h; ++h) {
      auto *raw = dstdata + h * dst_w;
      auto *src = srcdata + (h + roi.top) * src_w + roi.left;
      memcpy(raw, src, dst_w);
    }

    // copy uv
    auto uv_data = srcdata + src_h * src_w;
    auto dstuvdata = dstdata + dst_h * dst_w;
    for (int32_t h = 0; h < dst_h / 2; ++h) {
      auto *raw = dstuvdata + h * dst_w;
      auto *src = uv_data + (h + (roi.top / 2)) * src_w + roi.left;
      memcpy(raw, src, dst_w);
    }
    return dst;
  }

  auto ret = prepareBpuResizeParam(range_w, range_h, dst_w, dst_h);
  if (ret != 0) {
    return dst;
  }

  hbDNNTensor input_tensor;
  prepare_nv12_tensor_without_padding(src.data, src_h, src_w, &input_tensor);
  // Prepare output tensor
  hbDNNTensor output_tensor;
  prepare_nv12_tensor_without_padding(dst_h, dst_w, &output_tensor);

  // resize
  hbDNNResizeCtrlParam ctrl = {
      HB_BPU_CORE_0, 0, HB_DNN_RESIZE_TYPE_BILINEAR, 0, 0, 0, 0};
  hbDNNTaskHandle_t task_handle = nullptr;

  ret = 0;
  if (range_w == 0 || range_h == 0) {
    ret = hbDNNResize(
        &task_handle, &output_tensor, &input_tensor, nullptr, &ctrl);
  } else {
    ret = hbDNNResize(&task_handle, &output_tensor, &input_tensor, &roi, &ctrl);
  }

  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv crop"), "hbDNNResize failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return dst;
  }
  ret = hbDNNWaitTaskDone(task_handle, 0);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv crop"),
                 "hbDNNWaitTaskDone failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return dst;
  }
  hbDNNReleaseTask(task_handle);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv crop"), "release task failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return dst;
  }

  size_t size = dst_h * dst_w * 3 / 2;
  memcpy(dst.data, output_tensor.sysMem[0].virAddr, size);
  ret = hbSysFreeMem(&(input_tensor.sysMem[0]));
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv crop"),
                 "Free input_tensor mem failed!");
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return dst;
  }
  ret = hbSysFreeMem(&(output_tensor.sysMem[0]));
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv crop"),
                 "Free output_tensor mem failed!");
    return dst;
  }
  return dst;
}

int hobotcv_rotate(const cv::Mat &src, cv::Mat &dst, ROTATION_E rotation) {
  hobotcv_front hobotcv;
  int ret =
      hobotcv.prepareRotateParam(src.cols, src.rows * 2 / 3, (int)rotation);
  if (ret != 0) {
    return -1;
  }
  hobotcv.src_w = src.cols;
  hobotcv.src_h = src.rows * 2 / 3;
  hobotcv.dst_w = src.cols;
  hobotcv.dst_h = src.rows * 2 / 3;

  ret = hobotcv.groupScheduler();
  if (ret != 0) {
    return -1;
  }
  ret = hobotcv.sendVpsFrame(src);
  if (ret != 0) {
    return -1;
  }
  ret = hobotcv.getChnFrame(dst);
  if (ret != 0) {
    return -1;
  }
  return 0;
}

int hobotcv_imgproc(const cv::Mat &src,
                    cv::Mat &dst,
                    int dst_h,
                    int dst_w,
                    ROTATION_E rotate,
                    const cv::Range &rowRange,
                    const cv::Range &colRange) {
  int src_h = src.rows * 2 / 3;
  int src_w = src.cols;

  hobotcv_front hobotcv;
  int ret =
      hobotcv.prepareCropRoi(src_h, src_w, dst_w, dst_h, rowRange, colRange);
  if (ret != 0) {
    return -1;
  }

  ret = hobotcv.prepareResizeParam(src_w, src_h, dst_w, dst_h);
  if (0 != ret) {
    return -1;
  }

  ret = hobotcv.prepareRotateParam(dst_w, dst_h, (int)rotate);
  if (ret != 0) {
    return -1;
  }
  if (hobotcv.roi.height == dst_h && hobotcv.roi.width == dst_w &&
      hobotcv.rotate == 0) {  // only crop
    auto srcdata = reinterpret_cast<const uint8_t *>(src.data);
    dst = cv::Mat(dst_h * 3 / 2, dst_w, CV_8UC1);
    auto dstdata = dst.data;
    // copy y
    for (int h = 0; h < dst_h; ++h) {
      auto *raw = dstdata + h * dst_w;
      auto *src = srcdata + (h + hobotcv.roi.y) * src_w + hobotcv.roi.x;
      memcpy(raw, src, dst_w);
    }

    // copy uv
    auto uv_data = srcdata + src_h * src_w;
    auto dstuvdata = dstdata + dst_h * dst_w;
    for (int32_t h = 0; h < dst_h / 2; ++h) {
      auto *raw = dstuvdata + h * dst_w;
      auto *src = uv_data + (h + (hobotcv.roi.y / 2)) * src_w + hobotcv.roi.x;
      memcpy(raw, src, dst_w);
    }
    return 0;
  }

  ret = hobotcv.groupScheduler();
  if (ret != 0) {
    return -1;
  }
  ret = hobotcv.sendVpsFrame(src);
  if (ret != 0) {
    return -1;
  }
  ret = hobotcv.getChnFrame(dst);
  if (ret != 0) {
    return -1;
  }
  return 0;
}

int hobotcv_pymscale(const cv::Mat &src,
                     OutputPyramid *output,
                     const PyramidAttr &attr) {
  int src_h = src.rows * 2 / 3;
  int src_w = src.cols;
  hobotcv_front hobotcv;
  auto ret = hobotcv.preparePymraid(src_h, src_w, attr);
  if (0 != ret) {
    return -1;
  }

  ret = hobotcv.groupScheduler();
  if (ret != 0) {
    return -1;
  }
  ret = hobotcv.sendVpsFrame(src);
  if (ret != 0) {
    return -1;
  }
  ret = hobotcv.getPyramidOutputImage(output);
  if (ret != 0) {
    return -1;
  }
  return 0;
}

bool check_padding_area(const PaddingArea &area) {
  if (area.top == 0 && area.bottom == 0 && area.left == 0 && area.right == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv padding"), "No padding area!");
    return false;
  } else if (area.top % 2 != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv padding"),
                 "Invalid top size: %d! Padding size must be even");
    return false;
  } else if (area.bottom % 2 != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv padding"),
                 "Invalid bottom size: %d! Padding size must be even");
    return false;
  } else if (area.left % 2 != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv padding"),
                 "Invalid left size: %d! Padding size must be even");
    return false;
  } else if (area.right % 2 != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv padding"),
                 "Invalid right size: %d! Padding size must be even");
    return false;
  }
  return true;
}

void hobotcv_constant_padding(const char *src,
                              const int &src_h,
                              const int &src_w,
                              char *dst,
                              const PaddingArea &area,
                              uint8_t value) {
  int dst_w = src_w + area.left + area.right;
  int dst_h = src_h + area.top + area.bottom;
  int dst_y_size = dst_w * dst_h;
  int dst_uv_size = dst_w * dst_h / 2;
  char *dst_y = dst;
  char *dst_uv = dst + dst_y_size;
  memset(dst, value, dst_y_size + dst_uv_size);

  // padding y
  for (int h = 0; h < src_h; ++h) {
    auto *raw = dst_y + (h + area.top) * dst_w + area.left;
    auto *src_y_raw = src + h * src_w;
    memcpy(raw, src_y_raw, src_w);
  }

  // padding uv
  auto *src_uv_data = src + src_h * src_w;
  for (int h = 0; h < src_h / 2; ++h) {
    auto *raw = dst_uv + (h + (area.top / 2)) * dst_w + area.left;
    auto *src_uv_raw = src_uv_data + h * src_w;
    memcpy(raw, src_uv_raw, src_w);
  }
}

void hobotcv_replicate_padding(const char *src,
                               const int &src_h,
                               const int &src_w,
                               char *dst,
                               const PaddingArea &area) {
  uint32_t dst_w = src_w + area.left + area.right;
  uint32_t dst_h = src_h + area.top + area.bottom;
  int dst_y_size = dst_w * dst_h;
  char *dst_y = dst;
  char *dst_uv = dst + dst_y_size;
  auto *src_uv = src + src_h * src_w;
  uint32_t dst_bottom_start_line = src_h + area.top;
  // padding top and bottom
  for (uint32_t h = 0; h < dst_h; ++h) {  // padding y
    auto *raw = dst_y + h * dst_w + area.left;
    // auto *src_y_raw;
    if (h < area.top) {  // padding top
      auto *src_y_raw = src + h * src_w;
      memcpy(raw, src_y_raw, src_w);
    } else if (h >= dst_bottom_start_line) {  // padding bottom
      int bottom_line = h - dst_bottom_start_line;
      auto *src_y_raw = src + (src_h - area.bottom + bottom_line) * src_w;
      memcpy(raw, src_y_raw, src_w);
    } else {  // copy src
      auto *src_y_raw = src + (h - area.top) * src_w;
      memcpy(raw, src_y_raw, src_w);
    }
  }

  for (uint32_t h = 0; h < dst_h / 2; ++h) {  // padding uv
    auto *raw = dst_uv + h * dst_w + area.left;
    if (h < (area.top / 2)) {  // top
      auto *src_uv_raw = src_uv + h * src_w;
      memcpy(raw, src_uv_raw, src_w);
    } else if (h >= (dst_bottom_start_line / 2)) {  // bottom
      int bottom_line = h - (dst_bottom_start_line / 2);
      auto *src_uv_raw =
          src_uv + ((src_h / 2) - (area.bottom / 2) + bottom_line) * src_w;
      memcpy(raw, src_uv_raw, src_w);
    } else {  // copy src
      auto *src_uv_raw = src_uv + (h - (area.top / 2)) * src_w;
      memcpy(raw, src_uv_raw, src_w);
    }
  }

  // padding left and right
  for (uint32_t h = 0; h < dst_h; ++h) {
    // padding left
    auto *dst_left_y = dst_y + h * dst_w;
    auto *dst_left_uv = dst_uv + (h / 2) * dst_w;
    auto *src_left_y = dst_y + h * dst_w + area.left;
    auto *src_left_uv = dst_uv + (h / 2) * dst_w + area.left;
    memcpy(dst_left_y, src_left_y, area.left);
    memcpy(dst_left_uv, src_left_uv, area.left);
    // padding right
    auto *dst_right_y = dst_y + (h + 1) * dst_w - area.right;
    auto *dst_right_uv = dst_uv + (1 + (h / 2)) * dst_w - area.right;
    auto *src_right_y = dst_y + (h + 1) * dst_w - area.right * 2;
    auto *src_right_uv = dst_uv + (1 + (h / 2)) * dst_w - area.right * 2;
    memcpy(dst_right_y, src_right_y, area.right);
    memcpy(dst_right_uv, src_right_uv, area.right);
  }
}

int hobotcv_BorderPadding(const char *src,
                          const int &src_h,
                          const int &src_w,
                          char *dst,
                          const Hobotcv_Padding_Type type,
                          const PaddingArea &area,
                          const uint8_t value) {
  if (!check_padding_area(area)) {
    return -1;
  }
  if (type == HOBOTCV_CONSTANT) {
    hobotcv_constant_padding(src, src_h, src_w, dst, area, value);
  } else if (type == HOBOTCV_REPLICATE) {
    hobotcv_replicate_padding(src, src_h, src_w, dst, area);
  } else {
    return -1;
  }
  return 0;
}

}  // namespace hobot_cv
