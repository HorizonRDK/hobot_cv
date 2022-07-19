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

#ifndef HOBOT_CV_INCLUDE_HOBOTCV_IMGPROC_HPP_
#define HOBOT_CV_INCLUDE_HOBOTCV_IMGPROC_HPP_

#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"

namespace hobot_cv {

typedef enum HOBOT_CV_ROTATION_E {
  ROTATION_0 = 0,
  ROTATION_90,
  ROTATION_180,
  ROTATION_270,
  ROTATION_MAX
} ROTATION_E;

enum HOBOT_CV_TYPE { HOBOTCV_BPU = 0, HOBOTCV_VPS };

int hobotcv_resize(HOBOT_CV_TYPE type,
                   const cv::Mat &src,
                   cv::Mat &dst,
                   int dst_h,
                   int dst_w,
                   const cv::Range &rowRange,
                   const cv::Range &colRange);

int hobotcv_rotate(const cv::Mat &src, cv::Mat &dst, ROTATION_E rotate);

int hobotcv_imgproc(const cv::Mat &src,
                    cv::Mat &dst,
                    int dst_h,
                    int dst_w,
                    ROTATION_E rotate,
                    const cv::Range &rowRange,
                    const cv::Range &colRange);

}  // namespace hobot_cv

#endif  // HOBOT_CV_INCLUDE_HOBOTCV_IMGPROC_HPP_
