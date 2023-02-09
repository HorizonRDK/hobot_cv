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

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <queue>
#include <utility>

#include "dnn/hb_dnn.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "hobotcv_gaussian_blur_imp.hpp"

namespace hobotcv {

enum VLOG_LEVEL {
  EXAMPLE_SYSTEM = 0,
  EXAMPLE_REPORT = 1,
  EXAMPLE_DETAIL = 2,
  EXAMPLE_DEBUG = 3
};

#define HB_CHECK_SUCCESS(value, errmsg)                              \
  do {                                                               \
    auto ret_code = value;                                           \
    if (ret_code != 0) {                                             \
      std::cout << errmsg << ", error code:" << ret_code << std::endl; \
      return ret_code;                                               \
    }                                                                \
  } while (0);

int32_t CopyMat2Tensor(cv::Mat &m_tof, hbDNNTensor *input_tensor) {
  if (m_tof.empty()) {
    std::cout << "image file not exist!" << std::endl;
    return -1;
  }
  hbDNNTensor *input = input_tensor;
  hbDNNTensorProperties Properties = input->properties;
  // int tensor_id = 0;
  int input_h = Properties.validShape.dimensionSize[1];
  int input_w = Properties.validShape.dimensionSize[2];
  // int input_c = Properties.alignedShape.dimensionSize[3];
  if (Properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    input_h = Properties.validShape.dimensionSize[2];
    input_w = Properties.validShape.dimensionSize[3];
    // input_c = Properties.alignedShape.dimensionSize[1];
  }

  if ((input_h != m_tof.rows) || (input_w != m_tof.cols)) {
    HB_CHECK_SUCCESS(((input_h == m_tof.rows) && (input_w == m_tof.cols)),
                     "read tof file resolution error");
    return -1;
  }

  // copy tof data
  char *data_ptr1 = reinterpret_cast<char *>(input->sysMem[0].virAddr);
  cv::Mat mat_tensor(input_h, input_w, CV_32SC1, data_ptr1);
  m_tof.convertTo(mat_tensor, CV_32SC1);

  HB_CHECK_SUCCESS(hbSysFlushMem(&input->sysMem[0], HB_SYS_MEM_CACHE_CLEAN),
                   "hbSysFlushMem failed");

  return 0;
}

HobotGaussianBlur::HobotGaussianBlur(const HobotGaussianBlurParam para) {
  assert(PlatformCheck());
  if (!(para.type == HOBOTCV_BPU || para.type == HOBOTCV_AUTO)) {
    std::cout << "unsupported calc type" << std::endl;
    throw std::runtime_error("error");
  }

  std::string module_name =
      "gaussian_blur_" + std::to_string(para.width) + "_" +
      std::to_string(para.height) + "_" + std::to_string(para.ksizeX) + "_" +
      std::to_string(para.ksizeY) + "_" + std::to_string(para.sigmaX) + "_" +
      std::to_string(para.sigmaY) + ".hbm";
  std::string module_file = "config/models/" + module_name;
  auto file_name = module_file.c_str();

  int ret = -1;
  ret = hbDNNInitializeFromFiles(&packed_dnn_handle, &file_name, 1);
  if (ret) {
    std::cout << "hbDNNInitializeFromFiles failed" << std::endl;
    throw std::runtime_error("error");
  }
  const char **model_name_list;
  int model_count = 0;

  ret =
      hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle);
  if (ret) {
    std::cout << "hbDNNGetModelNameList failed" << std::endl;
    throw std::runtime_error("error");
  }
  ret = hbDNNGetModelHandle(&dnn_handle, packed_dnn_handle, model_name_list[0]);
  if (ret) {
    std::cout << "hbDNNGetModelHandle failed" << std::endl;
    throw std::runtime_error("error");
  }
  ret = hbDNNGetInputCount(&input_count, dnn_handle);
  if (ret) {
    std::cout << "hbDNNGetInputCount failed" << std::endl;
    throw std::runtime_error("error");
  }
  ret = hbDNNGetOutputCount(&output_count, dnn_handle);
  if (ret) {
    std::cout << "hbDNNGetOutputCount failed" << std::endl;
    throw std::runtime_error("error");
  }
  input_tensors.resize(input_count);
  output_tensors.resize(output_count);

  ret = PrepareTensor();
  if (ret) {
    std::cout << "PrepareTensor failed" << std::endl;
    throw std::runtime_error("error");
  }
}
HobotGaussianBlur::~HobotGaussianBlur() {
  for (int i = 0; i < input_count; i++) {
    if (hbSysFreeMem(&(input_tensors[i].sysMem[0]))) {
      std::cout << "hbSysFreeMem failed" << std::endl;
    }
  }

  for (int i = 0; i < output_count; i++) {
    if (hbSysFreeMem(&(output_tensors[i].sysMem[0]))) {
      std::cout << "hbSysFreeMem failed" << std::endl;
    }
  }
  if (hbDNNRelease(packed_dnn_handle)) {
    std::cout << "hbDNNRelease failed" << std::endl;
  }
}

int HobotGaussianBlur::GaussianBlur(cv::Mat &src, cv::Mat &dst) {
  // 前处理
  HB_CHECK_SUCCESS(CopyMat2Tensor(src, input_tensors.data()),
                   "CopyMat2Tensor failed");

  // 推理
  hbDNNTaskHandle_t task_handle = nullptr;
  hbDNNTensor *output = output_tensors.data();
  hbDNNInferCtrlParam infer_ctrl_param;
  HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
  HB_CHECK_SUCCESS(hbDNNInfer(&task_handle, &output, input_tensors.data(),
                              dnn_handle, &infer_ctrl_param),
                   "hbDNNInfer failed");
  HB_CHECK_SUCCESS(hbDNNWaitTaskDone(task_handle, 0),
                   "hbDNNWaitTaskDone failed");

  // 后处理
  for (int i = 0; i < output_count; i++) {
    HB_CHECK_SUCCESS(hbSysFlushMem(&output_tensors[i].sysMem[0],
                                   HB_SYS_MEM_CACHE_INVALIDATE),
                     "hbSysFlushMem failed");
  }

  HB_CHECK_SUCCESS(GetResult(output, dst), "GetResult failed");
  HB_CHECK_SUCCESS(hbDNNReleaseTask(task_handle), "hbDNNReleaseTask failed");
  return 0;
}

bool HobotGaussianBlur::PlatformCheck() {
  auto version = hbDNNGetVersion();
  std::cout << "DNN version:" << version << std::endl;
  return true;
}

int HobotGaussianBlur::PrepareTensor(void) {
  hbDNNTensor *input = input_tensors.data();
  for (int i = 0; i < input_count; i++) {
    HB_CHECK_SUCCESS(
        hbDNNGetInputTensorProperties(&input[i].properties, dnn_handle, i),
        "hbDNNGetInputTensorProperties failed");

    int input_memSize = input[i].properties.alignedByteSize;
    // HB_CHECK_SUCCESS(hbSysAllocMem(&input[i].sysMem[0], input_memSize),
    //                  "hbSysAllocMem failed");
    HB_CHECK_SUCCESS(hbSysAllocCachedMem(&input[i].sysMem[0], input_memSize),
                     "hbSysAllocCachedMem failed");
  }

  hbDNNTensor *output = output_tensors.data();
  for (int i = 0; i < output_count; i++) {
    HB_CHECK_SUCCESS(
        hbDNNGetOutputTensorProperties(&output[i].properties, dnn_handle, i),
        "hbDNNGetOutputTensorProperties failed");
    int output_memSize = output[i].properties.alignedByteSize;
    // HB_CHECK_SUCCESS(hbSysAllocMem(&output[i].sysMem[0], output_memSize),
    //                  "hbSysAllocMem failed");
    HB_CHECK_SUCCESS(hbSysAllocCachedMem(&output[i].sysMem[0], output_memSize),
                     "hbSysAllocCachedMem failed");
  }

  return 0;
}

int HobotGaussianBlur::GetResult(hbDNNTensor *tensor,
                                 cv::Mat &mat_filter_short) {
  HB_CHECK_SUCCESS(
      hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE),
      "hbSysFlushMem failed");
  int input_h = tensor->properties.validShape.dimensionSize[1];
  int input_w = tensor->properties.validShape.dimensionSize[2];
  if (tensor->properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    input_h = tensor->properties.validShape.dimensionSize[2];
    input_w = tensor->properties.validShape.dimensionSize[3];
  }
  cv::Mat mat_tensor(input_h, input_w, CV_32SC1,
      reinterpret_cast<int *>(tensor->sysMem[0].virAddr));
  mat_tensor.convertTo(mat_filter_short, CV_16UC1);

  return 0;
}

}  // namespace hobotcv
