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

#ifndef HOBOTCV_GAUSSIAN_BLUR_HPP_
#define HOBOTCV_GAUSSIAN_BLUR_HPP_

#include "opencv2/core/mat.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#define HOBOTCV_INITIALIZE_GAUSSIAN_BLUR_PARAM(param) \
  {                                                   \
    (param)->type = HOBOTCV_BPU;                      \
    (param)->width = 320;                             \
    (param)->height = 240;                            \
    (param)->ksizeX = 3;                              \
    (param)->ksizeY = 3;                              \
    (param)->sigmaX = 0;                              \
    (param)->sigmaY = 0;                              \
  }

/**
 * @brief 支持的计算单元
 */
typedef enum HobotCalType_ {
  HOBOTCV_CPU,
  HOBOTCV_BPU,
  HOBOTCV_DSP,
  HOBOTCV_GPU,
  HOBOTCV_AUTO
} HobotCalType;

/**
 * @brief 高斯模糊初始化参数
 */
typedef struct HobotGaussianBlurParam_ {
  HobotCalType type;  // 计算单元类型，目前只支持 HOBOTCV_BPU
  int width;          // 处理图片宽，目前只支持 320
  int height;         // 处理图片高，目前只支持 240
  int ksizeX;         // 高斯核X大小，目前只支持 3
  int ksizeY;         // 高斯核Y大小，目前只支持 3
  double sigmaX;      // sigmaX，目前只支持 0
  double sigmaY;      // sigmaY，目前只支持 0
} HobotGaussianBlurParam;

typedef void *HobotCVGaussianBlurHandle;

/**
 * @brief 创建句柄
 * @param param [in] 创建句柄参数
 * @param phandle [out] 返回的句柄
 * @return int 0表示成功，<0则为错误码
 */
int HobotCVGaussianBlurCreate(HobotGaussianBlurParam param,
                              HobotCVGaussianBlurHandle *phandle);

/**
 * @brief 高斯模糊处理
 * @param handle [in] 句柄
 * @param src [in] 输入Mat数据
 * @param dst [out] 输出Mat数据
 * @param para [in] 处理参数，需要和创建句柄时的一致
 * @return int 0表示成功，<0则为错误码
 */
int HobotCVGaussianBlurProcess(HobotCVGaussianBlurHandle handle, cv::Mat *src,
                               cv::Mat *dst);

/**
 * @brief 销毁句柄
 * @param handle [in] sdk 句柄
 * @return int 0表示成功，<0则为错误码
 */
int HobotCVGaussianBlurDestroy(HobotCVGaussianBlurHandle handle);

#ifdef __cplusplus
}
#endif

#endif  // HOBOTCV_GAUSSIAN_BLUR_HPP_
