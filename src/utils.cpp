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

#include "include/utils.h"

#include <arm_neon.h>

#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

int prepareBpuResizeParam(int src_w, int src_h, int dst_w, int dst_h) {
  if (src_w % 16 != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv resize"),
                 "unsupported src width %d! The src width must "
                 "be a multiple of 16!",
                 src_w);
    return -1;
  }
  if (src_h % 2 != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv resize"),
                 "unsupported src height %d! The src height must be even!",
                 src_h);
    return -1;
  }
  if (dst_w % 16 != 0) {
    int remain = dst_w % 16;
    int recommend_dst_width = dst_w + 16 - remain;
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv resize"),
                 "unsupported dst width %d! The dst width must "
                 "be a multiple of 16! The recommended dst width is %d ",
                 dst_w,
                 recommend_dst_width);
    return -1;
  }
  if (dst_h % 2 != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv resize"),
                 "unsupported dst height %d! The dst height must be even!",
                 dst_h);
    return -1;
  }
  if (src_w > 4080 || src_h > 4080) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv resize"),
                 "unsupported src resolution %d x %d ! The src resolution must "
                 "be less than 4080 x 4080 !",
                 src_w,
                 src_h);
    return -1;
  }
  if (dst_w > 4080 || dst_h > 4080) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv resize"),
                 "unsupported dst resolution %d x %d ! The dst resolution must "
                 "be less than 4080 x 4080 !",
                 dst_w,
                 dst_h);
    return -1;
  }
  if (dst_w > src_w * 256) {
    float width_ratio = static_cast<float>(dst_w) / static_cast<float>(src_w);
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv resize"),
                 "Max 256x upscale is supported! dst width: %d src width: "
                 "%d width ratio: %f. Please change the src or dst width",
                 dst_w,
                 src_w,
                 width_ratio);
    return -1;
  }
  if (dst_h > src_h * 256) {
    float height_ratio = static_cast<float>(dst_h) / static_cast<float>(src_h);
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv resize"),
                 "Max 256x upscale is supported! dst height: %d src height: "
                 "%d height ratio: %f. Please change the src or dst height",
                 dst_h,
                 src_h,
                 height_ratio);
    return -1;
  }

  if (dst_w < src_w / 185) {
    float width_ratio = static_cast<float>(dst_w) / static_cast<float>(src_w);
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv resize"),
                 "Max 1/185 downscale is supported! dst width: %d src width: "
                 "%d width ratio: %f. Please change the src or dst width",
                 dst_w,
                 src_w,
                 width_ratio);
    return -1;
  }

  if (dst_h < src_h / 185) {
    float height_ratio = static_cast<float>(dst_h) / static_cast<float>(src_h);
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv resize"),
                 "Max 1/185 downscale is supported! dst height: %d src height: "
                 "%d height ratio: %f. Please change the src or dst height",
                 dst_h,
                 src_h,
                 height_ratio);
    return -1;
  }

  return 0;
}

void prepare_nv12_tensor_without_padding(uint8_t *image_data,
                                         int image_height,
                                         int image_width,
                                         hbDNNTensor *tensor) {
  auto &properties = tensor->properties;
  properties.tensorType = HB_DNN_IMG_TYPE_NV12;
  properties.tensorLayout = HB_DNN_LAYOUT_NCHW;
  auto &valid_shape = properties.validShape;
  valid_shape.numDimensions = 4;
  valid_shape.dimensionSize[0] = 1;
  valid_shape.dimensionSize[1] = 3;
  valid_shape.dimensionSize[2] = image_height;
  valid_shape.dimensionSize[3] = image_width;

  auto &aligned_shape = properties.alignedShape;
  aligned_shape = valid_shape;

  int32_t image_length = image_height * image_width * 3 / 2;

  hbSysAllocCachedMem(&tensor->sysMem[0], image_length);
  memcpy(tensor->sysMem[0].virAddr, image_data, image_length);

  hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_CLEAN);
}

void prepare_nv12_tensor_without_padding(int image_height,
                                         int image_width,
                                         hbDNNTensor *tensor) {
  auto &properties = tensor->properties;
  properties.tensorType = HB_DNN_IMG_TYPE_NV12;
  properties.tensorLayout = HB_DNN_LAYOUT_NCHW;

  auto &valid_shape = properties.validShape;
  valid_shape.numDimensions = 4;
  valid_shape.dimensionSize[0] = 1;
  valid_shape.dimensionSize[1] = 3;
  valid_shape.dimensionSize[2] = image_height;
  valid_shape.dimensionSize[3] = image_width;

  auto &aligned_shape = properties.alignedShape;
  int32_t w_stride = ALIGN_16(image_width);
  aligned_shape.numDimensions = 4;
  aligned_shape.dimensionSize[0] = 1;
  aligned_shape.dimensionSize[1] = 3;
  aligned_shape.dimensionSize[2] = image_height;
  aligned_shape.dimensionSize[3] = w_stride;

  int32_t image_length = image_height * w_stride * 3 / 2;
  hbSysAllocCachedMem(&tensor->sysMem[0], image_length);
}

int32_t BGRToNv12(cv::Mat &bgr_mat, cv::Mat &img_nv12) {
  auto height = bgr_mat.rows;
  auto width = bgr_mat.cols;

  if (height % 2 || width % 2) {
    std::cerr << "input img height and width must aligned by 2!";
    return -1;
  }
  cv::Mat yuv_mat;
  cv::cvtColor(bgr_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);
  if (yuv_mat.data == nullptr) {
    std::cerr << "yuv_mat.data is null pointer" << std::endl;
    return -1;
  }

  auto *yuv = yuv_mat.ptr<uint8_t>();
  if (yuv == nullptr) {
    std::cerr << "yuv is null pointer" << std::endl;
    return -1;
  }
  img_nv12 = cv::Mat(height * 3 / 2, width, CV_8UC1);
  auto *ynv12 = img_nv12.ptr<uint8_t>();

  int32_t uv_height = height / 2;
  int32_t uv_width = width / 2;

  // copy y data
  int32_t y_size = height * width;
  memcpy(ynv12, yuv, y_size);

  // copy uv data
  uint8_t *nv12 = ynv12 + y_size;
  uint8_t *u_data = yuv + y_size;
  uint8_t *v_data = u_data + uv_height * uv_width;

  for (int32_t i = 0; i < uv_width * uv_height; i++) {
    *nv12++ = *u_data++;
    *nv12++ = *v_data++;
  }
  return 0;
}

uint64_t currentMicroseconds() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}
