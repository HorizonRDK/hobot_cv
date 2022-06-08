// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef HOBOT_CV_INCLUDE_UTILS_HPP_
#define HOBOT_CV_INCLUDE_UTILS_HPP_

#include <memory>
#include <string>

#include "dnn/hb_dnn.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#define ALIGNED_2E(w, alignment) \
  ((static_cast<uint32_t>(w) + (alignment - 1U)) & (~(alignment - 1U)))
#define ALIGN_4(w) ALIGNED_2E(w, 4U)
#define ALIGN_8(w) ALIGNED_2E(w, 8U)
#define ALIGN_16(w) ALIGNED_2E(w, 16U)
#define ALIGN_64(w) ALIGNED_2E(w, 64U)

void prepare_nv12_tensor_without_padding(uint8_t *image_data,
                                        int image_height,
                                        int image_width,
                                        hbDNNTensor *tensor);

void prepare_nv12_tensor_without_padding(int image_height,
                          int image_width,
                          hbDNNTensor *tensor);

int32_t BGRToNv12(cv::Mat &bgr_mat, cv::Mat &img_nv12);


#endif  // HOBOT_CV_INCLUDE_UTILS_HPP_
