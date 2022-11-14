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
#ifndef HOBOTCV_FRONT_H
#define HOBOTCV_FRONT_H

#include "hobotcv_imgproc/hobotcv_single.h"

namespace hobot_cv {

// hobotcv padding功能填充区域合规检查
bool check_padding_area(uint32_t top,
                        uint32_t bottom,
                        uint32_t left,
                        uint32_t right,
                        const int &src_h,
                        const int &src_w,
                        int padding_type);

/* hobotcv constant填充方式，用传入的value填充，
成功返回填充后图片数据，失败返回nullptr*/
std::unique_ptr<char[]> hobotcv_constant_padding(const char *src,
                                                 const int &src_h,
                                                 const int &src_w,
                                                 uint32_t top,
                                                 uint32_t bottom,
                                                 uint32_t left,
                                                 uint32_t right,
                                                 uint8_t value);

/* hobotcv replicate填充方式，复制最边界像素填充
成功返回填充后图片数据，失败返回nullptr*/
std::unique_ptr<char[]> hobotcv_replicate_padding(const char *src,
                                                  const int &src_h,
                                                  const int &src_w,
                                                  uint32_t top,
                                                  uint32_t bottom,
                                                  uint32_t left,
                                                  uint32_t right);

/* hobotcv reflect填充方式，已原图边界为轴镜像填充
成功返回填充后图片数据，失败返回nullptr*/
std::unique_ptr<char[]> hobotcv_reflect_padding(const char *src,
                                                const int &src_h,
                                                const int &src_w,
                                                uint32_t top,
                                                uint32_t bottom,
                                                uint32_t left,
                                                uint32_t right);

int hobotcv_vps_resize(const cv::Mat &src,
                       cv::Mat &dst,
                       int dst_h,
                       int dst_w,
                       const cv::Range &rowRange,
                       const cv::Range &colRange);

hbSysMem *hobotcv_vps_resize(const char *src,
                             const int src_h,
                             const int src_w,
                             int &dst_h,
                             int &dst_w,
                             const cv::Range &rowRange,
                             const cv::Range &colRange);

class hobotcv_front {
 public:
  explicit hobotcv_front();
  ~hobotcv_front();

  int prepareResizeParam(int src_width,
                         int src_height,
                         int dst_width,
                         int dst_height,
                         bool printLog = true);

  int prepareRotateParam(int width, int height, int rotation);

  int prepareCropRoi(int src_height,
                     int src_width,
                     int dst_width,
                     int dst_height,
                     const cv::Range &rowRange,
                     const cv::Range &colRange,
                     bool printLog = true);

  int preparePymraid(int src_height, int src_width, const PyramidAttr &attr);

  int groupScheduler();

  int setVpsChannelAttr();

  int sendVpsFrame(const char *src, int src_h, int src_w);

  int getChnFrame(cv::Mat &dst);

  hbSysMem *getChnFrame(int &dst_h, int &dst_w);

  int getPyramidOutputImage(OutputPyramid *output);

 private:
  int createGroup();
  int setChannelAttr(int enscale);
  int setChannelRotate();
  int setChannelPyramidAttr();
  int group_sem_wait();
  int group_sem_post();
  //初始化channel后才支持channel的动态设置
  int groupChn1Init(int group_id, int max_w, int max_h);    // pym channel
  int groupChn2Init(int group_id, int max_w, int max_h);    // down scale Init
  int groupChn5Init(int group_id, int max_w, int max_h);    // up scale init
  int groupPymChnInit(int group_id, int max_w, int max_h);  // pyramid init

  int copyOutputImage(int stride,
                      int width,
                      int height,
                      address_info_t &img_addr,
                      void *output);

 public:
  int src_w;
  int src_h;
  int dst_w;
  int dst_h;
  int rotate = 0;
  CropRect roi;
  PyramidParam pym_param;
  int group_id;
  int channel_id;

 private:
  hobotcv_single *observe;
  int processId = 0;
  int ds_layer_en = 0;
};

}  // namespace hobot_cv

#endif  // HOBOTCV_FRONT_H
