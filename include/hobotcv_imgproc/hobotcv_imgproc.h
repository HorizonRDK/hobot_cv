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

#ifndef HOBOT_CV_INCLUDE_HOBOTCV_IMGPROC_HPP_
#define HOBOT_CV_INCLUDE_HOBOTCV_IMGPROC_HPP_

#define PYM_MAX_DS_LENGTH (2048 * 2048 * 3 / 2)
#define PYM_MAX_US_LENGTH (4096 * 4096 * 3 / 2)

#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"

namespace hobot_cv {

typedef enum HOBOT_CV_ROTATION_E {
  ROTATION_0 = 0,
  ROTATION_90,
  ROTATION_180,
  ROTATION_270,
  ROTATION_MAX
} ROTATION_E;

typedef struct HOBOT_CV_PYM_DS_INFO {
  int width;
  int height;
  char img[PYM_MAX_DS_LENGTH];
} PymDsInfo;

typedef struct HOBOT_CV_PYM_US_INFO {
  int width;
  int height;
  char img[PYM_MAX_US_LENGTH];
} PymUsInfo;

typedef struct HOBOT_CV_PYRAMID_OUTPUT {
  bool isSuccess;
  PymDsInfo pym_ds[6];  // down scale output
} OutputPyramid;

struct HOBOT_CV_PYM_SCALE_INFO {
  uint8_t factor;       //缩放参数（0~63）
  uint16_t roi_x;       //起始x坐标
  uint16_t roi_y;       //起始y坐标
  uint16_t roi_width;   //图像宽
  uint16_t roi_height;  //图像高
};

typedef struct HOBOT_CV_PYM_ATTR {
  int timeout;
  uint16_t ds_layer_en;  //取值范围 4~23
  HOBOT_CV_PYM_SCALE_INFO ds_info[24];
} PyramidAttr;

enum HobotcvSpeedUpType { HOBOTCV_AUTO = 0, HOBOTCV_VPS = 1, HOBOTCV_BPU = 2 };

int hobotcv_resize(const cv::Mat &src,
                   int src_h,
                   int src_w,
                   cv::Mat &dst,
                   int dst_h,
                   int dst_w,
                   HobotcvSpeedUpType type = HOBOTCV_AUTO);

cv::Mat hobotcv_crop(const cv::Mat &src,
                     int src_h,
                     int src_w,
                     int dst_h,
                     int dst_w,
                     const cv::Range &rowRange,
                     const cv::Range &colRange,
                     HobotcvSpeedUpType type = HOBOTCV_AUTO);

int hobotcv_rotate(const cv::Mat &src, cv::Mat &dst, ROTATION_E rotate);

int hobotcv_imgproc(const cv::Mat &src,
                    cv::Mat &dst,
                    int dst_h,
                    int dst_w,
                    ROTATION_E rotate,
                    const cv::Range &rowRange,
                    const cv::Range &colRange);

int hobotcv_pymscale(const cv::Mat &src,
                     OutputPyramid *output,
                     const PyramidAttr &attr);

}  // namespace hobot_cv

#endif  // HOBOT_CV_INCLUDE_HOBOTCV_IMGPROC_HPP_
