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

#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "hobotcv_imgproc/hobotcv_imgproc.h"
#include "include/utils.h"

namespace hobot_cv {

int hobotcv_resize(const cv::Mat &src,
                   int src_h,
                   int src_w,
                   cv::Mat &dst,
                   int dst_h,
                   int dst_w,
                   HobotcvSpeedUpType type) {

  cv::Mat y_src = cv::Mat(src_h, src_w, CV_8UC1);
  cv::Mat u_src = cv::Mat(src_h / 2, src_w / 2, CV_8UC1);
  cv::Mat v_src = cv::Mat(src_h / 2, src_w / 2, CV_8UC1);

  auto *yuv = src.ptr<uint8_t>();
  auto *y = y_src.ptr<uint8_t>();
  auto *u = u_src.ptr<uint8_t>();
  auto *v = v_src.ptr<uint8_t>();

  int32_t y_size = src_h * src_w;
  memcpy(y, yuv, y_size);
  yuv = yuv + y_size;
  for (int32_t i = 0; i < src_h / 4 * src_w; i++) {
    *u++ = *yuv++;
    *v++ = *yuv++;
  }
  
  cv::Mat y_dst;
  cv::Mat u_dst;
  cv::Mat v_dst;
  cv::resize(y_src, y_dst, cv::Size(dst_w, dst_h));
  cv::resize(u_src, u_dst, cv::Size(dst_w / 2, dst_h / 2));
  cv::resize(v_src, v_dst, cv::Size(dst_w / 2, dst_h / 2));
  
  cv::Mat dst_tmp(dst_h * 3 / 2, dst_w, CV_8UC1);
  auto *yuvout = dst_tmp.ptr<uint8_t>();
  auto *yout = y_dst.ptr<uint8_t>();
  auto *uout = u_dst.ptr<uint8_t>();
  auto *vout = v_dst.ptr<uint8_t>();

  int32_t yout_size = dst_h * dst_w;
  memcpy(yuvout, yout, yout_size);
  yuvout = yuvout + yout_size;

  for (int32_t i = 0; i < dst_h / 4 * dst_w; i++) {
    *yuvout++ = *uout++;
    *yuvout++ = *vout++;
  }
  dst = dst_tmp;


  // // 2. .
  // cv::resize(src, dst, cv::Size(dst_w, dst_h));
  
  std::cout << "finish resize" << std::endl;
  return 0;
}

}