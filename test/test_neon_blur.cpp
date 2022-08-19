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

#include "hobotcv_neon_blur.h"
#include "include/utils.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"

void analyse_result(cv::Mat &out_filter,
                    cv::Mat &cls_filter,
                    std::string flag_name) {
  std::cout << "" << std::endl;
  std::cout << "analyse_result start " << std::endl;
  std::cout << "---------" << flag_name << std::endl;
  double minvalue, maxvalue;
  cv::Point mixIdx, maxIdx;
  cv::minMaxLoc(out_filter, &minvalue, &maxvalue, &mixIdx, &maxIdx);
  cv::minMaxLoc(cls_filter, &minvalue, &maxvalue, &mixIdx, &maxIdx);

  cv::Mat mat_diff = cv::abs(out_filter - cls_filter);
  cv::Scalar sum_error = cv::sum(mat_diff >= 1);
  cv::Scalar mean_error = cv::sum(mat_diff) / (mat_diff.rows * mat_diff.cols);

  cv::minMaxLoc(mat_diff, &minvalue, &maxvalue, &mixIdx, &maxIdx);

  std::cout << "error sum:" << sum_error[0] << ",max:" << maxvalue
            << ",mean_error:" << mean_error[0] << std::endl;
  std::cout << "" << std::endl;
}

int main() {
  for (int i = 0; i < 5; i++) {
    std::string m_tof_file_s =
        "config/tof_images/frame1_" + std::to_string(i) + ".png";
    std::cout << "===================" << std::endl;
    std::cout << "image name :" << m_tof_file_s << std::endl;
    cv::Mat src = cv::imread(m_tof_file_s, CV_16UC1);
    cv::Mat hobotcv_output_mat, opencv_output_mat;
    auto start_time_infe = std::chrono::steady_clock::now();
    auto ret = hobot_cv::HobotMeanBlur(src, hobotcv_output_mat, cv::Size(3, 3));
    if (ret == -2) {
      RCLCPP_ERROR(rclcpp::get_logger("neon_mean"),
                   "Please run on the X3 platform!");
      continue;
    } else if (ret < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("neon_mean"), "Hobotcv MeanBlur failed!");
      continue;
    }
    int infe_time = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - start_time_infe)
                        .count();
    std::cout << "hobotcv mean cost time:" << infe_time << std::endl;

    auto start_time_mean = std::chrono::steady_clock::now();
    cv::blur(src, opencv_output_mat, cv::Size(3, 3));
    int mean_time = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - start_time_mean)
                        .count();
    std::cout << "opencv mean cost time:" << mean_time << std::endl;
    float save_rate =
        static_cast<float>((mean_time * 1.0 - infe_time * 1.0) / mean_time);
    std::cout << "hobotcv mean save rate:" << save_rate << std::endl;
    analyse_result(hobotcv_output_mat, opencv_output_mat, "Mean_Blur");

    start_time_infe = std::chrono::steady_clock::now();
    ret = hobot_cv::HobotGaussianBlur(src, hobotcv_output_mat, cv::Size(3, 3));
    if (ret == -2) {
      RCLCPP_ERROR(rclcpp::get_logger("neon_gaussian"),
                   "Please run on the X3 platform!");
      continue;
    } else if (ret < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("neon_gaussian"),
                   "Hobotcv GaussianBlur failed!");
      continue;
    }
    infe_time = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - start_time_infe)
                    .count();
    std::cout << "hobotcv gaussian cost time:" << infe_time << std::endl;

    auto start_time_gauss = std::chrono::steady_clock::now();
    cv::GaussianBlur(src, opencv_output_mat, cv::Size(3, 3), 0, 0);
    int guss_time = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - start_time_gauss)
                        .count();
    std::cout << "opencv gaussian cost time:" << guss_time << std::endl;
    save_rate =
        static_cast<float>((guss_time * 1.0 - infe_time * 1.0) / guss_time);
    std::cout << "hobotcv gaussian save rate:" << save_rate << std::endl;
    analyse_result(hobotcv_output_mat, opencv_output_mat, "Gaussian_Blur");
    std::cout << "-------------------------" << std::endl;
  }

  return 0;
}
