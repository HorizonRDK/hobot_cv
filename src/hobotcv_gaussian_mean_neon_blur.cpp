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

#include "hobotcv_neon_blur.h"
#include "utils.h"

namespace hobot_cv {

int HobotGaussianBlur(const cv::Mat &src, cv::Mat &dst, cv::Size ksize) {
  int size_x = ksize.width;
  int size_y = ksize.height;
  if (!(size_x == 3 || size_x == 5) || !(size_y == 3 || size_y == 5)) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "Only support Size(3,3) and Size(5,5)");
    return -1;
  }

  int src_depth = src.depth();

  cv::Mat output;
  if (src_depth == CV_16SC1) {  // data type int16
    auto gaussion_output_ptr =
        (int16_t *)malloc(sizeof(int16_t) * src.rows * src.cols);
    output = cv::Mat(src.rows, src.cols, CV_16SC1, gaussion_output_ptr);
    if (size_x == 3 && size_y == 3) {  // Size(3x3)
      oal::gaussion_3x3_int16(
          (int16_t *)src.data, gaussion_output_ptr, src.cols, src.rows);
    } else if (size_x == 5 && size_y == 5) {  // Size(5x5)
      oal::gaussion_5x5_int16(
          (int16_t *)src.data, gaussion_output_ptr, src.cols, src.rows);
    }
    output.copyTo(dst);
    free(gaussion_output_ptr);
  } else if (src_depth == CV_16UC1) {  // data type uint16
    auto gaussion_output_ptr =
        (uint16_t *)malloc(sizeof(uint16_t) * src.rows * src.cols);
    output = cv::Mat(src.rows, src.cols, CV_16UC1, gaussion_output_ptr);
    if (size_x == 3 && size_y == 3) {  // Size(3x3)
      oal::gaussion_3x3(
          (uint16_t *)src.data, gaussion_output_ptr, src.cols, src.rows);
    } else if (size_x == 5 && size_y == 5) {  // Size(5x5)
      oal::gaussion_5x5(
          (uint16_t *)src.data, gaussion_output_ptr, src.cols, src.rows);
    }
    output.copyTo(dst);
    free(gaussion_output_ptr);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "Only support CV_16SC1 and CV_16UC1");
    return -1;
  }
  return 0;
}

int HobotMeanBlur(const cv::Mat &src, cv::Mat &dst, cv::Size ksize) {
  int size_x = ksize.width;
  int size_y = ksize.height;
  if (!(size_x == 3 || size_x == 5) || !(size_y == 3 || size_y == 5)) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "Only support Size(3,3) and Size(5,5)");
    return -1;
  }
  int src_depth = src.depth();

  cv::Mat output;
  if (src_depth == CV_16SC1) {  // data type int16
    auto mean_output_ptr =
        (int16_t *)malloc(sizeof(int16_t) * src.rows * src.cols);
    output = cv::Mat(src.rows, src.cols, CV_16SC1, mean_output_ptr);
    if (size_x == 3 && size_y == 3) {
      oal::mean_3x3_int16(
          (int16_t *)src.data, mean_output_ptr, src.cols, src.rows);
    } else if (size_x == 5 && size_y == 5) {
      oal::mean_5x5_int16(
          (int16_t *)src.data, mean_output_ptr, src.cols, src.rows);
    }
    output.copyTo(dst);
    free(mean_output_ptr);
  } else if (src_depth == CV_16UC1) {  // data type uint16
    auto mean_output_ptr =
        (uint16_t *)malloc(sizeof(uint16_t) * src.rows * src.cols);
    output = cv::Mat(src.rows, src.cols, CV_16UC1, mean_output_ptr);
    if (size_x == 3 && size_y == 3) {
      oal::mean_3x3((uint16_t *)src.data, mean_output_ptr, src.cols, src.rows);
    } else if (size_x == 5 && size_y == 5) {
      oal::mean_5x5((uint16_t *)src.data, mean_output_ptr, src.cols, src.rows);
    }
    output.copyTo(dst);
    free(mean_output_ptr);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "Only support CV_16SC1 and CV_16UC1");
    return -1;
  }

  return 0;
}

}  // namespace hobot_cv
