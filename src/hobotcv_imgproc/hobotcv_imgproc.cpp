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
#include "hobotcv_imgproc/hobotcv_service.h"
#include "include/utils.h"
#include "rclcpp/rclcpp.hpp"

namespace hobot_cv {

int hobotcv_resize(HOBOT_CV_TYPE type,
                   const cv::Mat &src,
                   cv::Mat &dst,
                   int dst_h,
                   int dst_w,
                   const cv::Range &rowRange,
                   const cv::Range &colRange) {
  int src_h = src.rows * 2 / 3;
  int src_w = src.cols;

  hobotcv_front hobotcv;
  if (type == HOBOTCV_BPU) {  // 调用BPU
    auto ret = hobotcv.hobotcv_bpu_resize(
        src, src_h, src_w, dst, dst_h, dst_w, rowRange, colRange);
    return ret;
  }

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
  hobotcv.shmfifoInit();
  int pid = -1;
  if (hobotcv.getServiceLaunched()) {
    ret = hobotcv.createInputImage(src);
    if (ret != 0) {
      return -1;
    }
    ret = hobotcv.getOutputImage(dst);
    if (ret != 0) {
      return -1;
    }
  } else {
    RCLCPP_WARN(rclcpp::get_logger("hobot_cv"), "launch hobot_cv service!");
    pid = fork();
    if (pid < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                   "hobot_cv service create failed!!");
      return -1;
    }
    if (pid == 0) {  // launch service
      hobot_cv::hobotcv_service service;
      service.serviceInit();
      service.groupScheduler();
    } else if (pid > 0) {
      hobotcv.setServiceLaunched(true);
      ret = hobotcv.createInputImage(src);
      if (ret != 0) {
        return -1;
      }
      ret = hobotcv.getOutputImage(dst);
      if (ret != 0) {
        return -1;
      }
    }
  }
  return 0;
}

int hobotcv_rotate(const cv::Mat &src, cv::Mat &dst, ROTATION_E rotation) {
  hobotcv_front hobotcv;
  int ret = hobotcv.prepareRotateParam((int)rotation);
  if (ret != 0) {
    return -1;
  }
  hobotcv.src_w = src.cols;
  hobotcv.src_h = src.rows * 2 / 3;
  hobotcv.dst_w = src.cols;
  hobotcv.dst_h = src.rows * 2 / 3;
  //获取input共享内存池
  hobotcv.shmfifoInit();
  int pid = -1;
  if (hobotcv.getServiceLaunched()) {
    ret = hobotcv.createInputImage(src);
    if (ret != 0) {
      return -1;
    }
    ret = hobotcv.getOutputImage(dst);
    if (ret != 0) {
      return -1;
    }
  } else {
    RCLCPP_WARN(rclcpp::get_logger("hobot_cv"), "launch hobot_cv service!");
    pid = fork();
    if (pid < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                   "hobot_cv service create failed!!");
      return -1;
    }
    if (pid == 0) {  // launch service
      hobot_cv::hobotcv_service service;
      service.serviceInit();
      service.groupScheduler();
    } else if (pid > 0) {
      hobotcv.setServiceLaunched(true);
      ret = hobotcv.createInputImage(src);
      if (ret != 0) {
        return -1;
      }
      ret = hobotcv.getOutputImage(dst);
      if (ret != 0) {
        return -1;
      }
    }
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

  ret = hobotcv.prepareRotateParam((int)rotate);
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

  //获取input共享内存池
  hobotcv.shmfifoInit();
  int pid = -1;
  if (hobotcv.getServiceLaunched()) {
    ret = hobotcv.createInputImage(src);
    if (ret != 0) {
      return -1;
    }
    ret = hobotcv.getOutputImage(dst);
    if (ret != 0) {
      return -1;
    }
  } else {
    RCLCPP_WARN(rclcpp::get_logger("hobot_cv"), "launch hobot_cv service!");
    pid = fork();
    if (pid < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                   "hobot_cv service create failed!!");
      return -1;
    }
    if (pid == 0) {  // launch service
      hobot_cv::hobotcv_service service;
      service.serviceInit();
      service.groupScheduler();
    } else if (pid > 0) {
      hobotcv.setServiceLaunched(true);
      ret = hobotcv.createInputImage(src);
      if (ret != 0) {
        return -1;
      }
      ret = hobotcv.getOutputImage(dst);
      if (ret != 0) {
        return -1;
      }
    }
  }
  return 0;
}

}  // namespace hobot_cv
