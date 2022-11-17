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
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#include "opencv2/core.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"

#ifndef HOBOTCV_BENCHMARK_NODE_H
#define HOBOTCV_BENCHMARK_NODE_H

using rclcpp::NodeOptions;

class hobotcv_benchmark_node : public rclcpp::Node {
 public:
  hobotcv_benchmark_node(
      const rclcpp::NodeOptions &node_options = NodeOptions(),
      std::string node_name = "cv_benchmark");
  ~hobotcv_benchmark_node();

  // hobot_cv resize
  void hobotcv_resize_benchmark(cv::Mat &src);

  // hobot_cv rotate
  void hobotcv_rotate_benchmark(cv::Mat &src);

  // opencv resize
  void opencv_resize_benchmark(cv::Mat &src);

  // opencv rotate
  void opencv_rotate_benchmark(cv::Mat &src);

 private:
  std::string image_file = "config/test.jpg";  //输入图片路径
  int src_width = 1920;                        //输入图片宽
  int src_height = 1080;                       //输入图片高
  int dst_height = 540;                        // 输出图片高
  int dst_width = 960;                         // 输出图片宽
  int rotation = 180;                          // 旋转角度

  std::string cv_type = "resize";  //图片操作type，resize/rotate
  int interface_type = 2;  // hobot_cv接口输入输出图片格式  1：cv::Mat   2: nv12
  std::string speed_type = "vps";  //图片处理加速类型  vps、bpu、opencv
};

#endif  // HOBOTCV_BENCHMARK_NODE_H