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

#define RED_COMMENT_START "\033[31m "
#define RED_COMMENT_END " \033[0m"
#define YELLOW_COMMENT_START "\033[33m "
#define YELLOW_COMMENT_END "\033[0m"

using rclcpp::NodeOptions;

enum class Speedup_Type { HOBOTCV_VPS = 0, HOBOTCV_BPU = 1, OPENCV = 2 };
enum class Process_Type { RESIZE = 0, ROTATE = 1 };
enum class Image_Format { MAT = 0, NV12 = 1 };

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

  void print_benchmark_log(std::string &method,
                           float min,
                           float max,
                           float total,
                           float fps_data,
                           int static_cycle);

 private:
  std::string image_file = "config/test.jpg";  //输入图片路径
  int src_width = 1920;                        //输入图片宽
  int src_height = 1080;                       //输入图片高
  int dst_height = 540;                        // 输出图片高
  int dst_width = 960;                         // 输出图片宽
  int rotation = 180;                          // 旋转角度

  int process_type_in = 0;  //图片操作type，0: resize 1:rotate
  int img_fmt_in = 0;  // hobot_cv接口输入输出图片格式  0：cv::Mat   1: nv12
  int speedup_type_in = 0;  //图片处理加速类型  0：vps、1: bpu、2: opencv

  Process_Type process_type = Process_Type::RESIZE;
  Image_Format img_fmt = Image_Format::MAT;
  Speedup_Type speed_type = Speedup_Type::HOBOTCV_VPS;
};

#endif  // HOBOTCV_BENCHMARK_NODE_H
