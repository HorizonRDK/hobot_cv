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
  auto start_time = std::chrono::steady_clock::now();
  std::cout << "" << std::endl;
  std::cout << "analyse_result start " << std::endl;
  std::cout << "---------" << flag_name << std::endl;
  std::cout << "out_filter type:" << out_filter.type()
            << ",cols:" << out_filter.cols << ",rows:" << out_filter.rows
            << ",channel:" << out_filter.channels() << std::endl;
  std::cout << "cls_filter type:" << cls_filter.type()
            << ",cols:" << cls_filter.cols << ",rows:" << cls_filter.rows
            << ",channel:" << cls_filter.channels() << std::endl;
  double minvalue, maxvalue;
  cv::Point mixIdx, maxIdx;
  cv::minMaxLoc(out_filter, &minvalue, &maxvalue, &mixIdx, &maxIdx);
  std::cout << "out_filter minvalue:" << minvalue << ",max:" << maxvalue
            << std::endl;
  std::cout << "out_filter min,x:" << mixIdx.x << ",y:" << mixIdx.y
            << std::endl;
  std::cout << "out_filter max,x:" << maxIdx.x << ",y:" << maxIdx.y
            << std::endl;

  cv::minMaxLoc(cls_filter, &minvalue, &maxvalue, &mixIdx, &maxIdx);
  std::cout << "cls_filter minvalue:" << minvalue << ",max:" << maxvalue
            << std::endl;
  std::cout << "cls_filter min,x:" << mixIdx.x << ",y:" << mixIdx.y
            << std::endl;
  std::cout << "cls_filter max,x:" << maxIdx.x << ",y:" << maxIdx.y
            << std::endl;

  cv::Mat mat_diff = cv::abs(out_filter - cls_filter);
  cv::Scalar sum_error = cv::sum(mat_diff >= 1);
  cv::Scalar mean_error = cv::sum(mat_diff) / (mat_diff.rows * mat_diff.cols);

  cv::minMaxLoc(mat_diff, &minvalue, &maxvalue, &mixIdx, &maxIdx);
  std::cout << "" << std::endl;
  std::cout << "diff diff diff" << std::endl;
  std::cout << "mat_diff minvalue:" << minvalue << ",max:" << maxvalue
            << std::endl;
  std::cout << "mat_diff min,x:" << mixIdx.x << ",y:" << mixIdx.y << std::endl;
  std::cout << "mat_diff max,x:" << maxIdx.x << ",y:" << maxIdx.y << std::endl;
  std::cout << "" << std::endl;

  std::cout << "error sum:" << sum_error[0] << ",max:" << maxvalue
            << ",mean_error:" << mean_error[0] << std::endl;

  int time_used_ms_end = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - start_time)
                             .count();
  std::cout << "analyse_result,time_used_ms_end:" << time_used_ms_end
            << std::endl;
  std::cout << "analyse_result end " << std::endl;
  std::cout << "" << std::endl;
}

int main() {
  for (int i = 0; i < 5; i++) {
    std::string m_tof_file_s =
        "config/tof_images/frame1_" + std::to_string(i) + ".png";

    std::cout << "===================" << std::endl;
    std::cout << "image name :" << m_tof_file_s << std::endl;
    cv::Mat src = cv::imread(m_tof_file_s, CV_16UC1);
    cv::Mat dst_mat, hobotcv_output_mat, opencv_output_mat;
    auto start_time_infe = std::chrono::steady_clock::now();
    hobot_cv::HobotGaussianBlur(src, dst_mat, cv::Size(3, 3));
    hobot_cv::HobotMeanBlur(dst_mat, hobotcv_output_mat, cv::Size(3, 3));
    int infe_time = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - start_time_infe)
                        .count();
    std::cout << "hobotcv cost time:" << infe_time << std::endl;

    auto start_time_gauss = std::chrono::steady_clock::now();
    cv::Mat gaussian_tof;
    cv::GaussianBlur(src, gaussian_tof, cv::Size(3, 3), 0, 0);
    cv::blur(gaussian_tof, opencv_output_mat, cv::Size(3, 3));
    int guss_time = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - start_time_gauss)
                        .count();
    std::cout << "opencv cost time:" << guss_time << std::endl;
    float save_rate =
        static_cast<float>((guss_time * 1.0 - infe_time * 1.0) / guss_time);
    std::cout << "hobotcv save rate:" << save_rate << std::endl;
    analyse_result(hobotcv_output_mat, opencv_output_mat, "Gaussian_mean_Blur");
    std::cout << "-------------------------" << std::endl;
  }
  // std::string image_file = "config/test.jpg";
  // cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
  // cv::Mat input_gray;
  // cv::cvtColor(bgr_mat, input_gray, cv::COLOR_RGB2GRAY);
  // cv::Mat input_s16;
  // input_gray.convertTo(input_s16, CV_16SC1);

  // cv::Mat input_u16;
  // input_gray.convertTo(input_u16, CV_16UC1);

  // RCLCPP_INFO(rclcpp::get_logger("test_neon_blur"),
  //             "input_s16.type: %d input_s16.depth: %d input_u16.type: %d "
  //             "input_u16.depth: %d",
  //             input_s16.type(),
  //             input_s16.depth(),
  //             input_u16.type(),
  //             input_u16.depth());

  // cv::Mat input_s16;
  // input_u16.convertTo(input_s16, CV_16SC1);

  // cv::Mat input_u16 = cv::imread(image_file, CV_16UC1);

  // cv::Mat input_s16 = cv::imread(image_file, CV_16SC1);

  // cv::Mat dst_mat;

  // auto ret = hobot_cv::HobotGaussianBlur(input_s16, dst_mat, cv::Size(3, 3));
  // dst_mat.convertTo(dst_mat, CV_8UC1);
  // cv::imwrite("./gaussion_3x3_int.png", dst_mat);

  // ret = hobot_cv::HobotGaussianBlur(input_s16, dst_mat, cv::Size(5, 5));
  // dst_mat.convertTo(dst_mat, CV_8UC1);
  // cv::imwrite("./gaussion_5x5_int.png", dst_mat);

  // ret = hobot_cv::HobotGaussianBlur(input_u16, dst_mat, cv::Size(3, 3));
  // dst_mat.convertTo(dst_mat, CV_8UC1);
  // cv::imwrite("./gaussion_3x3_uint.png", dst_mat);

  // ret = hobot_cv::HobotGaussianBlur(input_u16, dst_mat, cv::Size(5, 5));
  // dst_mat.convertTo(dst_mat, CV_8UC1);
  // cv::imwrite("./gaussion_5x5_uint.png", dst_mat);

  // ret = hobot_cv::HobotMeanBlur(input_s16, dst_mat, cv::Size(3, 3));
  // dst_mat.convertTo(dst_mat, CV_8UC1);
  // cv::imwrite("./mean_3x3_int.png", dst_mat);

  // ret = hobot_cv::HobotMeanBlur(input_s16, dst_mat, cv::Size(5, 5));
  // dst_mat.convertTo(dst_mat, CV_8UC1);
  // cv::imwrite("./mean_5x5_int.png", dst_mat);

  // ret = hobot_cv::HobotMeanBlur(input_u16, dst_mat, cv::Size(3, 3));
  // dst_mat.convertTo(dst_mat, CV_8UC1);
  // cv::imwrite("./mean_3x3_uint.png", dst_mat);

  // ret = hobot_cv::HobotMeanBlur(input_u16, dst_mat, cv::Size(5, 5));
  // dst_mat.convertTo(dst_mat, CV_8UC1);
  // cv::imwrite("./mean_5x5_uint.png", dst_mat);

  return 0;
}
