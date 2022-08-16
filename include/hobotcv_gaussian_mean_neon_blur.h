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
#ifndef HOBOTCV_GAUSSIAN_MEAN_NEON_BLUR_HPP_
#define HOBOTCV_GAUSSIAN_MEAN_NEON_BLUR_HPP_

#include "oal_gaussion.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hobot_cv {

int HobotGaussianBlur(const cv::Mat &src, cv::Mat &dst, cv::Size ksize);

int HobotMeanBlur(const cv::Mat &src, cv::Mat &dst, cv::Size ksize);

}  // namespace hobot_cv

#endif  // HOBOTCV_GAUSSIAN_MEAN_NEON_BLUR_HPP_
