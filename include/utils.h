// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef HOBOTCV_INCLUDE_HOBOTCV_UTILS_HPP_
#define HOBOTCV_INCLUDE_HOBOTCV_UTILS_HPP_

#include <memory>
#include <string>

#include "dnn/hb_dnn.h"

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


#endif  // HOBOTCV_INCLUDE_HOBOTCV_UTILS_HPP_
