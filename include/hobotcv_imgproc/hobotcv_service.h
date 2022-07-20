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
#ifndef HOBOTCV_SERVICE_H
#define HOBOTCV_SERVICE_H

#include <condition_variable>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "hobotcv_front.h"
#include "utils.h"
#include "vio/hb_vio_interface.h"
#include "vio/hb_vp_api.h"
#include "vio/hb_vps_api.h"

namespace hobot_cv {

typedef struct HOBOT_CV_CHANNEL_INFO {
  int channel_id;
  int output_w;
  int output_h;
  int rotation;
} channel_info;

typedef struct HOBOT_CV_GROUP_INFO {
  int group_id;
  uint64_t active_time;
  int max_w;
  int max_h;

  uint64_t mmz_paddr[2];
  char *mmz_vaddr[2];

  std::mutex input_list_mtx;                  //线程互斥锁
  std::condition_variable cv_list_not_empty;  //不为空唤醒work线程
  std::list<ShmInput_t *> input_list;
  std::map<int, std::shared_ptr<channel_info> > channel_map;  // key = chn_id
  std::shared_ptr<std::thread> work_thread;
} group_info;

class hobotcv_service {
 public:
  hobotcv_service(){};
  ~hobotcv_service(){};

  // service初始化，映射共享内存
  int serviceInit();

  int groupScheduler();

  std::shared_ptr<group_info> createGroup(ShmInput_t *input);

  //初始化channel后才支持channel的动态设置
  int groupChn0Init(int group_id, int max_w, int max_h);

  int groupChn1Init(int group_id, int max_w, int max_h);

  int groupChn2Init(int group_id, int max_w, int max_h);

  int groupChn5Init(int group_id, int max_w, int max_h);

  int setChannelAttr(
      int group_id, int chn_id, int width, int height, int enScale);

  int setchannelCrop(VPS_CROP_INFO_S &cropInfo, int group_id, int chn_id);

  int setChannelRotate(int group_id, int chn_id, int rotation);

  int sendVpsFrame(ShmInput_t *input, std::shared_ptr<group_info> group);

  int getChnFrame(int group_id,
                  int chn_id,
                  hb_vio_buffer_t &out_buf,
                  OutputImage *output);

 private:
  std::shared_ptr<std::thread> group_manager_thread;
  std::mutex group_list_mtx;
  std::list<std::shared_ptr<group_info> > group_list;
  int groupflag[8] = {0};
  hobot_cv::shmfifo_t fifo;
};

}  // namespace hobot_cv

#endif  // HOBOTCV_SERVICE_H
