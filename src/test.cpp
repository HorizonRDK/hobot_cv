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
#include "include/hobotcv_imgproc.h"
#include "include/utils.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include <fstream>
#include <string>
#include <chrono>
#include <iostream>

void writeImg(cv::Mat &mat, std::string imgfile)
{
  cv::Mat img_bgr;
  cv::cvtColor(mat, img_bgr, cv::COLOR_YUV2BGR_NV12);
  cv::imwrite(imgfile, img_bgr);
}

int main()
{
  std::string image_file = "config/test.jpg";
  cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
  auto src_height = bgr_mat.rows;
  auto src_width = bgr_mat.cols;
  
  cv::Mat srcmat_nv12;
  BGRToNv12(bgr_mat, srcmat_nv12);
  auto dst_height = src_height / 2;
  auto dst_width = src_width / 2;
  cv::Mat dstmat_nv12(dst_height * 3 / 2, dst_width, CV_8UC1);
  auto before_resize = std::chrono::system_clock::now();
  auto ret = hobot_cv::hobotcv_resize(srcmat_nv12, src_height, src_width, dstmat_nv12, dst_height, dst_width);
  auto after_resize = std::chrono::system_clock::now();
  auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        after_resize - before_resize).count();
  if(0 == ret) {
    std::cout << "resize finish, time: " << interval << "ms" << std::endl;
  }
  writeImg(dstmat_nv12, "./resize.jpg");

  auto before_crop = std::chrono::system_clock::now();
  auto cropmat =
    hobot_cv::hobotcv_crop(
      srcmat_nv12, src_height, src_width, 200, 200, cv::Range(0, 200), cv::Range(0, 200));
  auto after_crop = std::chrono::system_clock::now();
  interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        after_crop - before_crop).count();
  std::cout << "crop finish, time: " << interval << "ms" << std::endl;
  writeImg(cropmat, "./crop.jpg");

  auto before_cropResize = std::chrono::system_clock::now();
  auto cropResizemat =
    hobot_cv::hobotcv_crop(
      srcmat_nv12, src_height, src_width, src_height, src_width, cv::Range(200, 400), cv::Range(200, 400));
  auto after_cropResize = std::chrono::system_clock::now();
  interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        after_cropResize - before_cropResize).count();
  std::cout << "cropResize finish, time: " << interval << "ms" << std::endl;
  writeImg(cropResizemat, "./cropResize.jpg");

  return 0;
}
