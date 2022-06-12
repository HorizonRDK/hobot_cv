// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef HOBOTCV_INCLUDE_HOBOTCV_GAUSSIAN_BLUR_HPP_
#define HOBOTCV_INCLUDE_HOBOTCV_GAUSSIAN_BLUR_HPP_

#include "hobotcv_gaussian_blur.h"

#include <vector>

#include "dnn/hb_dnn.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

namespace hobotcv {

class HobotGaussianBlur {
 public:
  // 构造函数根据使用的计算资源和ksize大小完成初始化
  explicit HobotGaussianBlur(const HobotGaussianBlurParam para);
  // 析构函数完成资源释放
  ~HobotGaussianBlur();

  // 目前只支持固定大小的窗口以及固定sigma
  int GaussianBlur(cv::Mat &src, cv::Mat &dst);

 private:
  //运行平台检查，保证只能在我们的平台上运行，构造函数初始化中调用
  bool PlatformCheck(void);

  int PrepareTensor(void);
  int GetResult(hbDNNTensor *tensor, cv::Mat &mat_filter_short);

  int input_count = 0;
  int output_count = 0;

  hbPackedDNNHandle_t packed_dnn_handle;
  hbDNNHandle_t dnn_handle;
  std::vector<hbDNNTensor> input_tensors;
  std::vector<hbDNNTensor> output_tensors;
};

}  // namespace hobotcv

#endif  // HOBOTCV_INCLUDE_HOBOTCV_GAUSSIAN_BLUR_HPP_
