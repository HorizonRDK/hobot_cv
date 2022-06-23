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

#include "include/hobotcv_imgproc.h"

#include <iostream>

#include "include/utils.h"
#include "rclcpp/rclcpp.hpp"

namespace hobot_cv {

int hobotcv_resize(
    cv::Mat &src, int src_h, int src_w, cv::Mat &dst, int dst_h, int dst_w) {
  hbDNNTensor input_tensor;
  prepare_nv12_tensor_without_padding(src.data, src_h, src_w, &input_tensor);
  // Prepare output tensor
  hbDNNTensor output_tensor;
  prepare_nv12_tensor_without_padding(dst_h, dst_w, &output_tensor);
  // resize
  hbDNNResizeCtrlParam ctrl = {HB_BPU_CORE_0, 0, HB_DNN_RESIZE_TYPE_BILINEAR};
  hbDNNTaskHandle_t task_handle = nullptr;

  auto ret =
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

cv::Mat hobotcv_crop(cv::Mat &src,
                     int src_h,
                     int src_w,
                     int dst_h,
                     int dst_w,
                     const cv::Range &rowRange,
                     const cv::Range &colRange) {
  cv::Mat dst(dst_h * 3 / 2, dst_w, CV_8UC1);
  if (rowRange.end <= rowRange.start || colRange.end <= colRange.start ||
      rowRange.start < 0 || colRange.start < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "Invalid Range data! The end data must be greater than the "
                 "start data and the starting value cannot be less than zero!");
    return dst;
  }

  hbDNNRoi roi;
  roi.left = colRange.start;
  roi.top = rowRange.start;
  roi.right = colRange.end - 1;
  roi.bottom = rowRange.end - 1;
  auto range_h = rowRange.end - rowRange.start;
  auto range_w = colRange.end - colRange.start;
  if (range_h == dst_h && range_w == dst_w) {
    size_t size = dst_h * dst_w * 3 / 2;
    cv::Mat dst(dst_h * 3 / 2, dst_w, CV_8UC1);
    auto srcdata = reinterpret_cast<const uint8_t *>(src.data);
    auto dstdata = dst.data;
    // copy y
    for (int h = 0; h < dst_h; ++h) {
      auto *raw = dstdata + h * dst_w;
      auto *src = srcdata + h * src_w;
      memcpy(raw, src, dst_w);
    }

    // copy uv
    auto uv_data = srcdata + src_h * src_w;
    auto dstuvdata = dstdata + dst_h * dst_w;
    for (int32_t h = 0; h < dst_h / 2; ++h) {
      auto *raw = dstuvdata + h * dst_w;
      auto *src = uv_data + h * src_w;
      memcpy(raw, src, dst_w);
    }
    return dst;
  }

  hbDNNTensor input_tensor;
  prepare_nv12_tensor_without_padding(src.data, src_h, src_w, &input_tensor);
  // Prepare output tensor
  hbDNNTensor output_tensor;
  prepare_nv12_tensor_without_padding(dst_h, dst_w, &output_tensor);

  // resize
  hbDNNResizeCtrlParam ctrl = {HB_BPU_CORE_0, 0, HB_DNN_RESIZE_TYPE_BILINEAR};
  hbDNNTaskHandle_t task_handle = nullptr;

  int ret = 0;
  if (roi.right == 0 && roi.bottom == 0) {
    ret = hbDNNResize(
        &task_handle, &output_tensor, &input_tensor, nullptr, &ctrl);
  } else {
    ret = hbDNNResize(&task_handle, &output_tensor, &input_tensor, &roi, &ctrl);
  }

  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "hbDNNResize failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return dst;
  }
  ret = hbDNNWaitTaskDone(task_handle, 0);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "hbDNNWaitTaskDone failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return dst;
  }
  hbDNNReleaseTask(task_handle);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "release task failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return dst;
  }

  size_t size = dst_h * dst_w * 3 / 2;

  memcpy(dst.data, output_tensor.sysMem[0].virAddr, size);
  ret = hbSysFreeMem(&(input_tensor.sysMem[0]));
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "Free input_tensor mem failed!");
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return dst;
  }
  ret = hbSysFreeMem(&(output_tensor.sysMem[0]));
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "Free output_tensor mem failed!");
    return dst;
  }
  return dst;
}
}  // namespace hobot_cv
