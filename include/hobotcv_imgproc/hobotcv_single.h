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

#ifndef HOBOTCV_SINGLE_H
#define HOBOTCV_SINGLE_H

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

#include <map>
#include <memory>
#include <mutex>
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
  int enable;  //通道是否使能
  int output_w;
  int output_h;
  int rotation;
  int pym_enable;  //是否设置为pym 0:未设置 1：设置
} Channel_info_t;

typedef struct HOBOT_CV_GROUP_INFO {
  int group_id;  // group id
  int process_id;  // group绑定的进程id，每个group限定在一个固定进程内使用
  int group_state;  // group状态，是否已经被系统释放, 0:未被系统释放
                    // 1：已被系统释放
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

struct hobotcv_sys_mem {
  // vps系统内存
  uint64_t mmz_paddr[2];
  char *mmz_vaddr[2];
};

class hobotcv_single {
 public:
  ~hobotcv_single() {
    sem_wait(fifo.sem_groups);
    std::unique_lock<std::mutex> lk(group_map_mtx);
    auto group_it = group_map.begin();
    for (; group_it != group_map.end(); ++group_it) {
      int group_id = group_it->first;
      if (group_id >= HOBOTCV_GROUP_BEGIN) {
        Group_info_t *group =
            (Group_info_t *)(fifo.groups) + (group_id - HOBOTCV_GROUP_BEGIN);
        group->group_state = 1;
        group->process_id = 0;
        memset(group->channels, 0, sizeof(Channel_info_t) * 7);
      }
      HB_SYS_Free(group_it->second.mmz_paddr[0], group_it->second.mmz_vaddr[0]);
      HB_SYS_Free(group_it->second.mmz_paddr[1], group_it->second.mmz_vaddr[1]);
    }
    lk.unlock();
    sem_post(fifo.sem_groups);
    sem_close(fifo.sem_groups);
    sem_close(fifo.sem_group4);
    sem_close(fifo.sem_group5);
    sem_close(fifo.sem_group6);
    sem_close(fifo.sem_group7);
    shmdt(fifo.groups);
  }

  hobotcv_single(const hobotcv_single &) = delete;
  hobotcv_single &operator=(const hobotcv_single &) = delete;

  static hobotcv_single *getSingleObj() {
    static hobotcv_single obj;
    return &obj;
  }

  void Hobotcv_AddGroup(int group_id, hobotcv_sys_mem &sys_mem);
  hobotcv_sys_mem &GetGroupSysmem(int group_id);

  shmfifo_t fifo;

 private:
  std::mutex group_map_mtx;
  std::map<int, hobotcv_sys_mem> group_map;

  int shmfifoInit();

  hobotcv_single() {
    shmfifoInit();
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

}  // namespace hobot_cv

#endif  // HOBOTCV_SINGLE_H
