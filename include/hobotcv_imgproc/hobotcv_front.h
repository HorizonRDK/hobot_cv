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
#define HOBOTCV_GROUP_SIZE (4)
#define HOBOTCV_GROUP_BEGIN (4)
#define HOBOTCV_GROUP_OVER_TIME (10000000)

#include <fcntl.h>
#include <semaphore.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <memory>
#include <string>

#include "dnn/hb_dnn.h"
#include "hobotcv_imgproc/hobotcv_imgproc.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils.h"
#include "vio/hb_vio_interface.h"
#include "vio/hb_vp_api.h"
#include "vio/hb_vps_api.h"

namespace hobot_cv {

typedef struct HOBOT_CV_CHANNEL_INFO {
  int output_w;
  int output_h;
  int rotation;
  int pym_enable;
} Channel_info_t;

typedef struct HOBOT_CV_GROUP_INFO {
  int group_id;
  int process_id;   // 每个进程绑定一个固定group
  int group_state;  // group状态，是否已经被系统释放
  uint64_t active_time;
  int max_w;
  int max_h;
  Channel_info_t channels[7];
} Group_info_t;

typedef struct HOBOT_CV_SHM_FIFO {
  void *groups;
  int shmid;
  sem_t *sem_groups;  // group信息总互斥
  sem_t *sem_group4;  // group4互斥
  sem_t *sem_group5;  // group5互斥
  sem_t *sem_group6;  // group6互斥
  sem_t *sem_group7;  // group7互斥
} shmfifo_t;

typedef struct HOBOT_CV_CROP_RECT {
  int cropEnable;  // crop使能
  uint16_t x;
  uint16_t y;
  uint16_t width;
  uint16_t height;
} CropRect;

typedef struct HOBOT_CV_PYM_PARAM {
  int pymEnable;  //金字塔处理使能 0/1
  PyramidAttr attr;
} PyramidParam;

class hobotcv_single {
 public:
  ~hobotcv_single() {
    fifo.sem_groups = sem_open("/sem_allgroup", O_CREAT);
    fifo.groups = shmat(fifo.shmid, NULL, 0);
    if (fifo.groups == (void *)-1) {
      RCLCPP_ERROR(rclcpp::get_logger("hobotcv_single"),
                   "shmfifo shmat failed!!");
      sem_close(fifo.sem_groups);
      return;
    }
    sem_wait(fifo.sem_groups);
    if (group_id >= HOBOTCV_GROUP_BEGIN) {
      Group_info_t *group =
          (Group_info_t *)(fifo.groups) + (group_id - HOBOTCV_GROUP_BEGIN);
      group->group_state = 1;
    }
    sem_post(fifo.sem_groups);
    sem_close(fifo.sem_groups);
  }

  hobotcv_single(const hobotcv_single &) = delete;
  hobotcv_single &operator=(const hobotcv_single &) = delete;

  static hobotcv_single &getSingleObj() {
    static hobotcv_single obj;
    return obj;
  }

  shmfifo_t fifo;
  int group_id;

 private:
  hobotcv_single() {
    // init vp
    VP_CONFIG_S struVpConf;
    memset(&struVpConf, 0x00, sizeof(VP_CONFIG_S));
    struVpConf.u32MaxPoolCnt = 32;
    HB_VP_SetConfig(&struVpConf);
    int ret = HB_VP_Init();
    if (0 != ret) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "HB_VP_Init failed!!");
    }
  }
};

class hobotcv_front {
 public:
  hobotcv_front();
  ~hobotcv_front();

  int shmfifoInit();

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
  hobotcv_single &observe;
  int processId = 0;
  int ds_layer_en = 0;

  // vps系统内存
  uint64_t mmz_paddr[2];
  char *mmz_vaddr[2];
};

}  // namespace hobot_cv

#endif  // HOBOTCV_FRONT_H
