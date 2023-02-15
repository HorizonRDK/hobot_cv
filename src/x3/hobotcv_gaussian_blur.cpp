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

#include <iostream>

#include "hobotcv_gaussian_blur.h"
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
                                   cv::Mat *src,
                                   cv::Mat *dst) {
  if (!handle) {
    std::cerr << "input handle null!" << std::endl;
    return -1;
  }
  if (!src) {
    std::cerr << "input src null!" << std::endl;
    return -1;
  }
  if (!dst) {
    std::cerr << "input dst null!" << std::endl;
    return -1;
  }
  auto pgb = reinterpret_cast<hobotcv::HobotGaussianBlur *>(handle);

  return pgb->GaussianBlur(*src, *dst);
}

int32_t HobotCVGaussianBlurDestroy(HobotCVGaussianBlurHandle handle) {
  if (!handle) {
    std::cerr << "input handle null!" << std::endl;
    return -1;
  }
  auto pgb = reinterpret_cast<hobotcv::HobotGaussianBlur *>(handle);
  if (pgb) {
    delete pgb;
    pgb = nullptr;
  }

  return 0;
}
