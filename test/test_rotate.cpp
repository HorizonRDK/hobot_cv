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
  auto dst_height = src_height / 2;
  auto dst_width = src_width / 2;
  cv::Mat dstmat_nv12(dst_height * 3 / 2, dst_width, CV_8UC1);

  {  // rotate first
    cv::Mat rotate_nv12;
    auto before_rotate = std::chrono::system_clock::now();
    auto ret = hobot_cv::hobotcv_rotate(
        srcmat_nv12, rotate_nv12, hobot_cv::ROTATION_180);
    auto after_rotate = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        after_rotate - before_rotate)
                        .count();
    if (ret == 0) {
      std::stringstream ss_rotate;
      ss_rotate << "rotate image 180 "
                << ", time cost: " << interval << " ms"
                << "\n";
      RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_rotate.str().c_str());
    }
    writeImg(rotate_nv12, "./rotate.jpg");
  }
  {  // rotate second
    cv::Mat rotate_nv12;
    auto before_rotate = std::chrono::system_clock::now();
    auto ret = hobot_cv::hobotcv_rotate(
        srcmat_nv12, rotate_nv12, hobot_cv::ROTATION_180);
    auto after_rotate = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        after_rotate - before_rotate)
                        .count();
    if (ret == 0) {
      std::stringstream ss_rotate;
      ss_rotate << "second rotate image 180 "
                << ", time cost: " << interval << " ms"
                << "\n";
      RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_rotate.str().c_str());
    }
  }

  return 0;
}
