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


#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <queue>
#include <utility>

#include "hobotcv_gaussian_blur.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

void analyse_result(cv::Mat &out_filter, cv::Mat &cls_filter,
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
  int32_t ret = -1;

  HobotGaussianBlurParam blur_param;
  HOBOTCV_INITIALIZE_GAUSSIAN_BLUR_PARAM(&blur_param);
  HobotCVGaussianBlurHandle handle;
  ret = HobotCVGaussianBlurCreate(blur_param, &handle);
  if (ret < 0) {
    printf("HobotCVGaussianBlurCreate failed, error msg:%d\n", ret);
    return -1;
  }

  for (int i = 0; i < 5; i++) {
    std::string m_tof_file_s = "config/tof_images/frame1_" +
        std::to_string(i) + ".png";

    std::cout << "===================" << std::endl;
    std::cout << "image name :" << m_tof_file_s << std::endl;
    cv::Mat src = cv::imread(m_tof_file_s, CV_16UC1);
    cv::Mat dst;
    cv::medianBlur(src, src, 3);
    auto start_time_infe = std::chrono::steady_clock::now();
    ret = HobotCVGaussianBlurProcess(handle, &src, &dst);
    if (ret < 0) {
      printf("HobotCVGaussianBlurProcess failed, error msg:%d\n", ret);
      return -1;
    }
    int infe_time = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - start_time_infe).count();
    std::cout << "infe cost time:" << infe_time << std::endl;

    auto start_time_gauss = std::chrono::steady_clock::now();
    cv::Mat gaussian_tof;
    cv::GaussianBlur(src, gaussian_tof, cv::Size(3, 3), 0, 0,
        cv::BORDER_REPLICATE);
    int guss_time = std::chrono::duration_cast<
        std::chrono::microseconds>(
        std::chrono::steady_clock::now() - start_time_gauss).count();
    std::cout << "guss_time cost time:" << guss_time << std::endl;
    float save_rate =
        static_cast<float>((guss_time * 1.0 - infe_time * 1.0) / guss_time);
    std::cout << "hobotcv save rate:" << save_rate << std::endl;
    analyse_result(dst, gaussian_tof, "GaussianBlur");
    std::cout << "-------------------------" << std::endl;
    std::cout << "" << std::endl;
  }

  ret = HobotCVGaussianBlurDestroy(handle);
  if (ret < 0) {
    printf("HobotCVGaussianBlurDestroy failed, error msg:%d\n", ret);
    return -1;
  }

  return 0;
}
