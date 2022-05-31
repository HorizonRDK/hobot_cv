// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef HOBOTCV_INCLUDE_HOBOTCV_DNNRESIZE_HPP_
#define HOBOTCV_INCLUDE_HOBOTCV_DNNRESIZE_HPP_

#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "dnn/hb_dnn.h"

#define ALIGNED_2E(w, alignment) \
  ((static_cast<uint32_t>(w) + (alignment - 1U)) & (~(alignment - 1U)))
#define ALIGN_4(w) ALIGNED_2E(w, 4U)
#define ALIGN_8(w) ALIGNED_2E(w, 8U)
#define ALIGN_16(w) ALIGNED_2E(w, 16U)
#define ALIGN_64(w) ALIGNED_2E(w, 64U)

namespace hobotcv {

  int hobotcv_resize(cv::Mat &src, int src_h, int src_w,
                                cv::Mat &dst, int dst_h, int dst_w);

  cv::Mat hobotcv_crop(cv::Mat &src, int src_h, int src_w, int dst_h, int dst_w,
                const cv::Range& rowRange, const cv::Range& colRange);

}  // namespace hobotcv

#endif  // HOBOTCV_INCLUDE_HOBOTCV_DNNRESIZE_HPP_
