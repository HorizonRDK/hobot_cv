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
  std::string image_file = "config/480x270.jpg";
  cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
  auto src_height = bgr_mat.rows;
  auto src_width = bgr_mat.cols;

  cv::Mat srcmat_nv12;
  BGRToNv12(bgr_mat, srcmat_nv12);

  hobot_cv::PaddingArea paddingArea;
  paddingArea.top = 100;
  paddingArea.left = 100;
  paddingArea.right = 100;
  paddingArea.bottom = 100;

  auto dst_height = src_height + paddingArea.top + paddingArea.bottom;
  auto dst_width = src_width + paddingArea.left + paddingArea.right;

  {  // constant
    auto before_padding = std::chrono::system_clock::now();
    auto ptr = hobot_cv::hobotcv_BorderPadding(
        reinterpret_cast<const char *>(srcmat_nv12.data),
        src_height,
        src_width,
        hobot_cv::HobotcvPaddingType::HOBOTCV_CONSTANT,
        paddingArea);
    auto after_padding = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        after_padding - before_padding)
                        .count();
    if (ptr != nullptr) {
      std::stringstream ss_padding;
      ss_padding << src_width << " x " << src_height
                 << " hobot_cv constant padding "
                 << " top:" << paddingArea.top
                 << " bottom: " << paddingArea.bottom
                 << " left: " << paddingArea.left
                 << " right: " << paddingArea.right
                 << ", time cost: " << interval << " ms"
                 << "\n";
      RCLCPP_INFO(
          rclcpp::get_logger("example"), "%s", ss_padding.str().c_str());
      cv::Mat dstmat_nv12(
          dst_height * 3 / 2, dst_width, CV_8UC1, (void *)(ptr.get()));
      writeImg(dstmat_nv12, "./cv_constant_padding.jpg");
    }
  }

  {  // HOBOTCV_REPLICATE
    auto before_padding = std::chrono::system_clock::now();
    auto ptr = hobot_cv::hobotcv_BorderPadding(
        reinterpret_cast<const char *>(srcmat_nv12.data),
        src_height,
        src_width,
        hobot_cv::HobotcvPaddingType::HOBOTCV_REPLICATE,
        paddingArea);
    auto after_padding = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        after_padding - before_padding)
                        .count();
    if (ptr != nullptr) {
      std::stringstream ss_padding;
      ss_padding << src_width << " x " << src_height
                 << " hobot_cv replicate padding"
                 << " top:" << paddingArea.top
                 << " bottom: " << paddingArea.bottom
                 << " left: " << paddingArea.left
                 << " right: " << paddingArea.right
                 << ", time cost: " << interval << " ms"
                 << "\n";
      RCLCPP_INFO(
          rclcpp::get_logger("example"), "%s", ss_padding.str().c_str());
      cv::Mat dstmat_nv12(
          dst_height * 3 / 2, dst_width, CV_8UC1, (void *)(ptr.get()));
      writeImg(dstmat_nv12, "./cv_replicate_padding.jpg");
    }
  }

  {  // HOBOTCV_REFLECT
    auto before_padding = std::chrono::system_clock::now();
    auto ptr = hobot_cv::hobotcv_BorderPadding(
        reinterpret_cast<const char *>(srcmat_nv12.data),
        src_height,
        src_width,
        hobot_cv::HobotcvPaddingType::HOBOTCV_REFLECT,
        paddingArea);
    auto after_padding = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        after_padding - before_padding)
                        .count();
    if (ptr != nullptr) {
      std::stringstream ss_padding;
      ss_padding << src_width << " x " << src_height
                 << " hobot_cv reflect padding"
                 << " top:" << paddingArea.top
                 << " bottom: " << paddingArea.bottom
                 << " left: " << paddingArea.left
                 << " right: " << paddingArea.right
                 << ", time cost: " << interval << " ms"
                 << "\n";
      RCLCPP_INFO(
          rclcpp::get_logger("example"), "%s", ss_padding.str().c_str());
      cv::Mat dstmat_nv12(
          dst_height * 3 / 2, dst_width, CV_8UC1, (void *)(ptr.get()));
      writeImg(dstmat_nv12, "./cv_reflect_padding.jpg");
    }
  }

  {  // opencv constant
    cv::Mat dst_mat;
    auto before_padding = std::chrono::system_clock::now();
    cv::copyMakeBorder(bgr_mat,
                       dst_mat,
                       paddingArea.top,
                       paddingArea.bottom,
                       paddingArea.left,
                       paddingArea.right,
                       cv::BORDER_CONSTANT);
    auto after_padding = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        after_padding - before_padding)
                        .count();
    std::stringstream ss_padding;
    ss_padding << src_width << " x " << src_height << " opencv constant padding"
               << " top:" << paddingArea.top
               << " bottom: " << paddingArea.bottom
               << " left: " << paddingArea.left
               << " right: " << paddingArea.right << ", time cost: " << interval
               << " ms"
               << "\n";
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_padding.str().c_str());
    cv::imwrite("./opencv_constant.jpg", dst_mat);
  }

  {  // opencv REPLICATE
    cv::Mat dst_mat;
    auto before_padding = std::chrono::system_clock::now();
    cv::copyMakeBorder(bgr_mat,
                       dst_mat,
                       paddingArea.top,
                       paddingArea.bottom,
                       paddingArea.left,
                       paddingArea.right,
                       cv::BORDER_REPLICATE);
    auto after_padding = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        after_padding - before_padding)
                        .count();
    std::stringstream ss_padding;
    ss_padding << src_width << " x " << src_height
               << " opencv replicate padding"
               << " top:" << paddingArea.top
               << " bottom: " << paddingArea.bottom
               << " left: " << paddingArea.left
               << " right: " << paddingArea.right << ", time cost: " << interval
               << " ms"
               << "\n";
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_padding.str().c_str());
    cv::imwrite("./opencv_replicate.jpg", dst_mat);
  }

  {  // opencv REFLECT
    cv::Mat dst_mat;
    auto before_padding = std::chrono::system_clock::now();
    cv::copyMakeBorder(bgr_mat,
                       dst_mat,
                       paddingArea.top,
                       paddingArea.bottom,
                       paddingArea.left,
                       paddingArea.right,
                       cv::BORDER_REFLECT);
    auto after_padding = std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        after_padding - before_padding)
                        .count();
    std::stringstream ss_padding;
    ss_padding << src_width << " x " << src_height << " opencv reflect padding"
               << " top:" << paddingArea.top
               << " bottom: " << paddingArea.bottom
               << " left: " << paddingArea.left
               << " right: " << paddingArea.right << ", time cost: " << interval
               << " ms"
               << "\n";
    RCLCPP_INFO(rclcpp::get_logger("example"), "%s", ss_padding.str().c_str());
    cv::imwrite("./opencv_reflect.jpg", dst_mat);
  }
  return 0;
}
