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

class hobotcv_front {
 public:
  explicit hobotcv_front();
  ~hobotcv_front();

  int prepareResizeParam(int src_width,
                         int src_height,
                         int dst_width,
                         int dst_height,
                         bool printLog = true);

  int prepareRotateParam(int rotation);

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

  int sendVpsFrame(const cv::Mat &src);

  int getChnFrame(cv::Mat &dst);

  int getPyramidOutputImage(OutputPyramid *output);

 private:
  int createGroup();
  int setChannelAttr(int enscale);
  int setChannelRotate();
  int setChannelPyramidAttr();
  int group_sem_wait();
  int group_sem_post();
  //初始化channel后才支持channel的动态设置
  int groupChn0Init(int group_id, int max_w, int max_h);
  int groupChn1Init(int group_id, int max_w, int max_h);
  int groupChn2Init(int group_id, int max_w, int max_h);
  int groupChn5Init(int group_id, int max_w, int max_h);
  int groupPymChnInit(int group_id, int max_w, int max_h);

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

  // vps系统内存
  uint64_t mmz_paddr[2];
  char *mmz_vaddr[2];
};

}  // namespace hobot_cv

#endif  // HOBOTCV_FRONT_H
