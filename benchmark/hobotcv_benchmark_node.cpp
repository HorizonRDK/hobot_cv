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

#include "benchmark/hobotcv_benchmark_node.h"

#include "hobotcv_imgproc/hobotcv_imgproc.h"
#include "utils.h"

hobotcv_benchmark_node::hobotcv_benchmark_node(
    const rclcpp::NodeOptions &node_options, std::string node_name)
    : Node(node_name, node_options) {
  this->declare_parameter<std::string>("image_file", image_file);
  this->get_parameter<std::string>("image_file", image_file);

  this->declare_parameter<int>("static_cycle", static_cycle);
  this->get_parameter<int>("static_cycle", static_cycle);

  this->declare_parameter<int>("dst_width", dst_width);
  this->get_parameter<int>("dst_width", dst_width);

  this->declare_parameter<int>("dst_height", dst_height);
  this->get_parameter<int>("dst_height", dst_height);

  this->declare_parameter<int>("rotation", rotation);
  this->get_parameter("rotation", rotation);

  this->declare_parameter<int>("process_type", process_type_in);
  this->get_parameter<int>("process_type", process_type_in);
  if (process_type_in == 0) {
    process_type = Process_Type::RESIZE;
  } else if (process_type_in == 1) {
    process_type = Process_Type::ROTATE;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv benchmark"),
                 "unsupport process_type: %d! 0: resize 1:rotate",
                 process_type_in);
    return;
  }

  this->declare_parameter<int>("img_fmt", img_fmt_in);
  this->get_parameter<int>("img_fmt", img_fmt_in);
  if (img_fmt_in == 0) {
    img_fmt = Image_Format::MAT;
  } else if (img_fmt_in == 1) {
    img_fmt = Image_Format::NV12;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv benchmark"),
                 "unsupport img_fmt: %d! 0: cv::Mat 1:nv12",
                 img_fmt_in);
    return;
  }

  this->declare_parameter<int>("speedup_type", speedup_type_in);
  this->get_parameter<int>("speedup_type", speedup_type_in);
  if (speedup_type_in == 0) {
    speed_type = Speedup_Type::HOBOTCV_VPS;
  } else if (speedup_type_in == 1) {
    speed_type = Speedup_Type::HOBOTCV_BPU;
  } else if (speedup_type_in == 2) {
    speed_type = Speedup_Type::OPENCV;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv benchmark"),
                 "unsupport speedup_type: %d! 0: vps、1: bpu、2: opencv",
                 speedup_type_in);
    return;
  }

  std::stringstream ss;
  ss << "\nspeed_type: " << speedup_type_in << "\nimg_fmt: " << img_fmt_in
     << "\nprocess_type: " << process_type_in << "\nimage_file: " << image_file
     << "\ndst_width: " << dst_width << "\ndst_height: " << dst_height
     << "\nrotation: " << rotation;

  RCLCPP_WARN(rclcpp::get_logger("hobot_cv benchmark"), "%s", ss.str().c_str());

  cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
  src_height = bgr_mat.rows;
  src_width = bgr_mat.cols;
  cv::Mat srcmat_nv12;
  BGRToNv12(bgr_mat, srcmat_nv12);
  if (process_type == Process_Type::RESIZE) {
    if (speed_type == Speedup_Type::HOBOTCV_VPS ||
        speed_type == Speedup_Type::HOBOTCV_BPU) {
      hobotcv_resize_benchmark(srcmat_nv12);
    } else if (speed_type == Speedup_Type::OPENCV) {
      opencv_resize_benchmark(bgr_mat);
    }
  } else if (process_type == Process_Type::ROTATE) {
    if (speed_type == Speedup_Type::HOBOTCV_VPS) {
      hobotcv_rotate_benchmark(srcmat_nv12);
    } else if (speed_type == Speedup_Type::OPENCV) {
      opencv_rotate_benchmark(bgr_mat);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv benchmark"),
                   "Rotate supports only vps and opencv");
      return;
    }
  }
}

hobotcv_benchmark_node::~hobotcv_benchmark_node() {}

void hobotcv_benchmark_node::print_benchmark_log(std::string &method,
                                                 float min,
                                                 float max,
                                                 float total_latency,
                                                 float fps_data) {
  auto avg_data = (total_latency - max - min) / (static_cycle - 2);
  if (process_type == Process_Type::RESIZE) {
    std::cout << YELLOW_COMMENT_START << method << " resize " << src_width
              << "x" << src_height << " to " << dst_width << "x" << dst_height
              << " Throughput " << fps_data << "fps" << YELLOW_COMMENT_END
              << std::endl;

    std::cout << RED_COMMENT_START << method << " resize " << src_width << "x"
              << src_height << " to " << dst_width << "x" << dst_height
              << " latency: [avg: " << avg_data << "ms,  max: " << max
              << "ms,  min: " << min << "ms]" << RED_COMMENT_END << std::endl;
  } else if (process_type == Process_Type::ROTATE) {
    std::cout << YELLOW_COMMENT_START << method << " rotate " << src_width
              << "x" << src_height << " " << rotation << " Throughput "
              << fps_data << "fps" << YELLOW_COMMENT_END << std::endl;

    std::cout << RED_COMMENT_START << method << " rotate " << src_width << "x"
              << src_height << " " << rotation << " latency: [avg: " << avg_data
              << "ms,  max: " << max << "ms,  min: " << min << "ms]"
              << RED_COMMENT_END << std::endl;
  }
}

void hobotcv_benchmark_node::hobotcv_resize_benchmark(cv::Mat &src) {
  std::string method;
  hobot_cv::HobotcvSpeedUpType type;
  if (speed_type == Speedup_Type::HOBOTCV_VPS) {
    type = hobot_cv::HOBOTCV_VPS;
    if (img_fmt == Image_Format::MAT) {
      method = "hobotcv VPS mat";
    } else if (img_fmt == Image_Format::NV12) {
      method = "hobotcv VPS nv12";
    }
  } else if (speed_type == Speedup_Type::HOBOTCV_BPU) {
    type = hobot_cv::HOBOTCV_BPU;
    if (img_fmt == Image_Format::MAT) {
      method = "hobotcv BPU mat";
    } else if (img_fmt == Image_Format::NV12) {
      method = "hobotcv BPU nv12";
    }
  }

  cv::Mat resized_mat(dst_height, dst_width, src.type());
  // vps or bpu第一次resize不进入耗时统计
  if (img_fmt == Image_Format::MAT) {
    hobot_cv::hobotcv_resize(
        src, src_height, src_width, resized_mat, dst_height, dst_width, type);
  } else if (img_fmt == Image_Format::NV12) {  // nv12
    auto imageInfo =
        hobot_cv::hobotcv_resize(reinterpret_cast<const char *>(src.data),
                                 src_height,
                                 src_width,
                                 dst_height,
                                 dst_width,
                                 type);
  }

  float latency = 0.0;
  float min_inter = 10000.0, max_inter = 0.0;
  int index = 0;
  auto start_ts = std::chrono::steady_clock::now();
  auto end_ts = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    auto start = std::chrono::steady_clock::now();
    if (img_fmt == Image_Format::MAT) {
      hobot_cv::hobotcv_resize(
          src, src_height, src_width, resized_mat, dst_height, dst_width, type);
    } else if (img_fmt == Image_Format::NV12) {  // nv12
      auto imageInfo =
          hobot_cv::hobotcv_resize(reinterpret_cast<const char *>(src.data),
                                   src_height,
                                   src_width,
                                   dst_height,
                                   dst_width,
                                   type);
    }
    auto end = std::chrono::steady_clock::now();
    float interval =
        static_cast<float>(
            std::chrono::duration_cast<std::chrono::microseconds>(end - start)
                .count()) /
        1000.0;
    min_inter = min_inter > interval ? interval : min_inter;
    max_inter = max_inter > interval ? max_inter : interval;
    latency += interval;
    index++;
    if (index % static_cycle == 0) {
      end_ts = std::chrono::steady_clock::now();
      float total_interval =
          static_cast<float>(
              std::chrono::duration_cast<std::chrono::microseconds>(end_ts -
                                                                    start_ts)
                  .count()) /
          1000.0;
      auto fps_data = (1000.0 * static_cycle / total_interval);
      print_benchmark_log(method, min_inter, max_inter, latency, fps_data);
      min_inter = 10000.0;
      max_inter = 0.0;
      latency = 0.0;
      start_ts = std::chrono::steady_clock::now();
    }
  }
}

void hobotcv_benchmark_node::hobotcv_rotate_benchmark(cv::Mat &src) {
  hobot_cv::ROTATION_E cv_rotation = hobot_cv::ROTATION_0;
  if (rotation == 90) {
    cv_rotation = hobot_cv::ROTATION_90;
  } else if (rotation == 180) {
    cv_rotation = hobot_cv::ROTATION_180;
  } else if (rotation == 270) {
    cv_rotation = hobot_cv::ROTATION_270;
  }

  float latency = 0.0;
  cv::Mat rotate_mat;
  std::string method;
  if (img_fmt == Image_Format::MAT) {  // mat
    method = "hobotcv VPS mat";
    hobot_cv::hobotcv_rotate(src, rotate_mat, cv_rotation);
  } else if (img_fmt == Image_Format::NV12) {  // nv12
    method = "hobotcv VPS nv12";
    auto imageInfo =
        hobot_cv::hobotcv_rotate(reinterpret_cast<const char *>(src.data),
                                 src_height,
                                 src_width,
                                 cv_rotation);
  }

  float min_inter = 10000.0, max_inter = 0.0;
  int index = 0;
  auto start_ts = std::chrono::steady_clock::now();
  auto end_ts = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    auto start = std::chrono::steady_clock::now();
    if (img_fmt == Image_Format::MAT) {
      hobot_cv::hobotcv_rotate(src, rotate_mat, cv_rotation);
    } else if (img_fmt == Image_Format::NV12) {  // nv12
      auto imageInfo =
          hobot_cv::hobotcv_rotate(reinterpret_cast<const char *>(src.data),
                                   src_height,
                                   src_width,
                                   cv_rotation);
    }
    auto end = std::chrono::steady_clock::now();
    float interval =
        static_cast<float>(
            std::chrono::duration_cast<std::chrono::microseconds>(end - start)
                .count()) /
        1000.0;
    min_inter = min_inter > interval ? interval : min_inter;
    max_inter = max_inter > interval ? max_inter : interval;
    latency += interval;
    index++;
    if (index % static_cycle == 0) {
      end_ts = std::chrono::steady_clock::now();
      float total_interval =
          static_cast<float>(
              std::chrono::duration_cast<std::chrono::microseconds>(end_ts -
                                                                    start_ts)
                  .count()) /
          1000.0;
      auto fps_data = (1000.0 * static_cycle / total_interval);
      print_benchmark_log(method, min_inter, max_inter, latency, fps_data);
      min_inter = 10000.0;
      max_inter = 0.0;
      latency = 0.0;
      start_ts = std::chrono::steady_clock::now();
    }
  }
}

void hobotcv_benchmark_node::opencv_resize_benchmark(cv::Mat &src) {
  float latency = 0.0;
  cv::Mat resized_mat(dst_height, dst_width, src.type());
  std::string method = "opencv";
  float min_inter = 10000.0, max_inter = 0.0;
  int index = 0;
  auto start_ts = std::chrono::steady_clock::now();
  auto end_ts = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    auto start = std::chrono::steady_clock::now();
    cv::resize(src, resized_mat, resized_mat.size(), 0, 0);
    auto end = std::chrono::steady_clock::now();
    float interval =
        static_cast<float>(
            std::chrono::duration_cast<std::chrono::microseconds>(end - start)
                .count()) /
        1000.0;
    min_inter = min_inter > interval ? interval : min_inter;
    max_inter = max_inter > interval ? max_inter : interval;
    latency += interval;
    index++;
    if (index % static_cycle == 0) {
      end_ts = std::chrono::steady_clock::now();
      float total_interval =
          static_cast<float>(
              std::chrono::duration_cast<std::chrono::microseconds>(end_ts -
                                                                    start_ts)
                  .count()) /
          1000.0;
      auto fps_data = (1000.0 * static_cycle / total_interval);
      print_benchmark_log(method, min_inter, max_inter, latency, fps_data);
      min_inter = 10000.0;
      max_inter = 0.0;
      latency = 0.0;
      start_ts = std::chrono::steady_clock::now();
    }
  }
}

void hobotcv_benchmark_node::opencv_rotate_benchmark(cv::Mat &src) {
  int opencv_rotation = 0;
  if (rotation == 90) {
    opencv_rotation = 0;
  } else if (rotation == 180) {
    opencv_rotation = 1;
  } else if (rotation == 270) {
    opencv_rotation = 2;
  }
  std::string method = "opencv";
  float latency = 0.0;
  cv::Mat rotate_mat;
  float min_inter = 10000.0, max_inter = 0.0;
  int index = 0;
  auto start_ts = std::chrono::steady_clock::now();
  auto end_ts = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    auto start = std::chrono::steady_clock::now();
    cv::rotate(src, rotate_mat, opencv_rotation);
    auto end = std::chrono::steady_clock::now();
    float interval =
        static_cast<float>(
            std::chrono::duration_cast<std::chrono::microseconds>(end - start)
                .count()) /
        1000.0;
    min_inter = min_inter > interval ? interval : min_inter;
    max_inter = max_inter > interval ? max_inter : interval;
    latency += interval;
    index++;
    if (index % static_cycle == 0) {
      end_ts = std::chrono::steady_clock::now();
      float total_interval =
          static_cast<float>(
              std::chrono::duration_cast<std::chrono::microseconds>(end_ts -
                                                                    start_ts)
                  .count()) /
          1000.0;
      auto fps_data = (1000.0 * static_cycle / total_interval);
      print_benchmark_log(method, min_inter, max_inter, latency, fps_data);
      min_inter = 10000.0;
      max_inter = 0.0;
      latency = 0.0;
      start_ts = std::chrono::steady_clock::now();
    }
  }
}
