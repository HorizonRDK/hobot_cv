// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef HOBOT_CV_INCLUDE_HOBOTCV_IMGPROC_HPP_
#define HOBOT_CV_INCLUDE_HOBOTCV_IMGPROC_HPP_

#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"

namespace hobot_cv {

  int hobotcv_resize(cv::Mat &src, int src_h, int src_w,
                                cv::Mat &dst, int dst_h, int dst_w);

  cv::Mat hobotcv_crop(cv::Mat &src, int src_h, int src_w, int dst_h, int dst_w,
                const cv::Range& rowRange, const cv::Range& colRange);

}  // namespace hobot_cv

#endif  // HOBOT_CV_INCLUDE_HOBOTCV_IMGPROC_HPP_
