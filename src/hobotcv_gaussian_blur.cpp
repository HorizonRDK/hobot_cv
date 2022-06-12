// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "hobotcv_gaussian_blur.h"

#include <iostream>

#include "hobotcv_gaussian_blur_imp.hpp"

int32_t HobotCVGaussianBlurCreate(HobotGaussianBlurParam param,
                                  HobotCVGaussianBlurHandle *phandle) {
  if (phandle == nullptr) {
    return -1;
  }
  try {
    auto pgb = new hobotcv::HobotGaussianBlur(param);
    *phandle = reinterpret_cast<HobotCVGaussianBlurHandle>(pgb);
  } catch (...) {
    return -1;
  }
  return 0;
}

int32_t HobotCVGaussianBlurProcess(HobotCVGaussianBlurHandle handle,
                                   cv::Mat *src, cv::Mat *dst) {
  if (!handle || !src || !dst) {
    std::cout << "input handle null!" << std::endl;
    return -1;
  }
  auto pgb = reinterpret_cast<hobotcv::HobotGaussianBlur *>(handle);

  return pgb->GaussianBlur(*src, *dst);
}

int32_t HobotCVGaussianBlurDestroy(HobotCVGaussianBlurHandle handle) {
  if (!handle) {
    std::cout << "input handle null!" << std::endl;
    return -1;
  }
  auto pgb = reinterpret_cast<hobotcv::HobotGaussianBlur *>(handle);
  if (pgb) {
    delete pgb;
    pgb = nullptr;
  }

  return 0;
}
