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
#ifndef HOBOTCV_NEON_BLUR_HPP_
#define HOBOTCV_NEON_BLUR_HPP_

#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"

namespace hobot_cv {

/**
 * neon加速高斯滤波
 * @param[in] src: 需要进行高斯滤波的图像，支持int16和uint16数据格式
 * @param[in] ksize: 滤波核大小
 * @param[out] dst：高斯滤波后输出的图像
 * @return 成功返回0，失败返回非0
 */
int HobotGaussianBlur(const cv::Mat &src, cv::Mat &dst, cv::Size ksize);

/**
 * neon加速均值滤波
 * @param[in] src: 需要进行均值滤波的图像，支持int16和uint16数据格式
 * @param[in] ksize: 滤波核大小
 * @param[out] dst：均值滤波后输出的图像
 * @return 成功返回0，失败返回非0
 */
int HobotMeanBlur(const cv::Mat &src, cv::Mat &dst, cv::Size ksize);

}  // namespace hobot_cv

#endif  // HOBOTCV_NEON_BLUR_HPP_
