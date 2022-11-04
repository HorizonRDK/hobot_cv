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
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#include "hobotcv_imgproc/hobotcv_imgproc.h"
#include "include/utils.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"

void writeImg(cv::Mat &mat, std::string imgfile) {
  cv::Mat img_bgr;
  cv::cvtColor(mat, img_bgr, cv::COLOR_YUV2BGR_NV12);
  cv::imwrite(imgfile, img_bgr);
}

int main() {
  std::string image_file = "config/test.jpg";
  cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
  auto src_height = bgr_mat.rows;
  auto src_width = bgr_mat.cols;

  cv::Mat srcmat_nv12;
  BGRToNv12(bgr_mat, srcmat_nv12);
  auto dst_height = src_height + 40;
  auto dst_width = src_width + 40;
  hobot_cv::PaddingArea paddingArea;
  paddingArea.top = 20;
  paddingArea.left = 20;
  paddingArea.right = 20;
  paddingArea.bottom = 20;

  {  // constant
    auto before_padding = std::chrono::system_clock::now();
    auto ptr = hobot_cv::hobotcv_BorderPadding(
        reinterpret_cast<const char *>(srcmat_nv12.data),
        src_height,
        src_width,
        hobot_cv::HobotcvBorderPadding::HOBOTCV_CONSTANT,
        paddingArea,
        255);
    auto after_padding = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        after_padding - before_padding)
                        .count();
    if (ptr != nullptr) {
      std::stringstream ss_padding;
      ss_padding
          << src_width << " x " << src_height
          << " pix constant padding top: 20 bottom: 20 left: 20 right: 20"
          << ", time cost: " << interval << " ms"
          << "\n";
      RCLCPP_INFO(
          rclcpp::get_logger("example"), "%s", ss_padding.str().c_str());
      cv::Mat dstmat_nv12(
          dst_height * 3 / 2, dst_width, CV_8UC1, (void *)(ptr.get()));
      writeImg(dstmat_nv12, "./constant_padding.jpg");
    }
  }

  {  // HOBOTCV_REPLICATE
    auto before_padding = std::chrono::system_clock::now();
    auto ptr = hobot_cv::hobotcv_BorderPadding(
        reinterpret_cast<const char *>(srcmat_nv12.data),
        src_height,
        src_width,
        hobot_cv::HobotcvBorderPadding::HOBOTCV_REPLICATE,
        paddingArea);
    auto after_padding = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        after_padding - before_padding)
                        .count();
    if (ptr != nullptr) {
      std::stringstream ss_padding;
      ss_padding
          << src_width << " x " << src_height
          << " pix replicate padding top: 20 bottom: 20 left: 20 right: 20"
          << ", time cost: " << interval << " ms"
          << "\n";
      RCLCPP_INFO(
          rclcpp::get_logger("example"), "%s", ss_padding.str().c_str());
      cv::Mat dstmat_nv12(
          dst_height * 3 / 2, dst_width, CV_8UC1, (void *)(ptr.get()));
      writeImg(dstmat_nv12, "./replicate_padding.jpg");
    }
  }
  return 0;
}
