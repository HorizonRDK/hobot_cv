// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.
#include "include/hobotcv_dnnresize.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include <fstream>
#include <string>

void writeImg(cv::Mat &mat, std::string imgfile)
{
  cv::Mat img_bgr;
  cv::cvtColor(mat, img_bgr, cv::COLOR_YUV2BGR_NV12);
  cv::imwrite(imgfile, img_bgr);
}

int main()
{
  std::string filename = "config/960_544.nv12";
  std::ifstream ifs(filename, std::ios::in | std::ios::binary);
    if (!ifs) {
      return -1;
    }
  ifs.seekg(0, std::ios::end);
  int len = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  char *data = new char[len];
  ifs.read(data, len);
  cv::Mat srcmat(544 * 3 / 2, 960, CV_8UC1, data);
  cv::Mat dstmat(272 * 3 / 2, 480, CV_8UC1);
  hobotcv::hobotcv_resize(srcmat, 544, 960, dstmat, 272, 480);
  writeImg(dstmat, "./resize.jpg");

  auto cropmat =
    hobotcv::hobotcv_crop(
      srcmat, 544, 960, 200, 200, cv::Range(0, 200), cv::Range(0, 200));
  writeImg(cropmat, "./crop.jpg");

  auto cropResizemat =
    hobotcv::hobotcv_crop(
      srcmat, 544, 960, 544, 960, cv::Range(200, 400), cv::Range(200, 400));
  writeImg(cropResizemat, "./cropResize.jpg");

  return 0;
}
