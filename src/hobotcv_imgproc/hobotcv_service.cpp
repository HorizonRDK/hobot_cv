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
#include "hobotcv_imgproc/hobotcv_service.h"

#include <fstream>
#include <iostream>

#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hobot_cv {
int hobotcv_service::serviceInit() {
  memset(&fifo, 0, sizeof(hobot_cv::shmfifo_t));
  // Init sem
  //初始化services时，input共享内存已经由hobotcv创建
  int shmid = shmget((key_t)1234, 0, 0);
  fifo.shmid = shmid;
  //连接 shared memory operations
  fifo.p_shm = (hobot_cv::shmhead_t *)shmat(fifo.shmid, NULL, 0);
  if (fifo.p_shm == (hobot_cv::shmhead_t *)-1) {
    RCLCPP_ERROR(rclcpp::get_logger("hobotcv_service"),
                 "shmfifo shmat failed!!");
    return -1;
  }
  //+1指针偏移,得到有效负载起始地址
  fifo.p_payload = (char *)(fifo.p_shm + 1);
  fifo.sem_mutex = sem_open("/sem_input", O_CREAT);
  fifo.sem_full = sem_open("/sem_input_full", O_CREAT);
  fifo.sem_empty = sem_open("/sem_input_empty", O_CREAT);
  fifo.sem_output = sem_open("/sem_output_key", O_CREAT);

  // init vp
  VP_CONFIG_S struVpConf;
  memset(&struVpConf, 0x00, sizeof(VP_CONFIG_S));
  struVpConf.u32MaxPoolCnt = 32;
  HB_VP_SetConfig(&struVpConf);
  int ret = HB_VP_Init();
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobotcv_service"), "HB_VP_Init failed!!");
    return -2;
  }

  // 启动group检测线程
  group_manager_thread = std::make_shared<std::thread>([this] {
    while (1) {
      sleep(10);  //等待group启动接收图片数据
      auto now_time = currentMicroseconds();
      std::unique_lock<std::mutex> mtx(group_list_mtx);
      auto it = group_list.begin();
      for (; it != group_list.end();) {
        auto active_time = (*it)->active_time;
        if (now_time - active_time > 10000000) {
          std::stringstream over_time;
          over_time << "group: " << (*it)->group_id
                    << "is over time! active_time: " << active_time
                    << " now_time: " << now_time;
          RCLCPP_WARN(rclcpp::get_logger("hobotcv_service"),
                      "%s",
                      over_time.str().c_str());

          this->groupflag[(*it)->group_id] = 0;
          HB_VPS_StopGrp((*it)->group_id);
          HB_VPS_DestroyGrp((*it)->group_id);
          it = group_list.erase(it);
        } else {
          ++it;
        }
      }
      mtx.unlock();
    }
  });
  return 0;
}

int hobotcv_service::groupScheduler() {
  while (1) {
    sem_wait(fifo.sem_empty);
    sem_wait(fifo.sem_mutex);
    ShmInput_t *input =
        (ShmInput_t *)(fifo.p_payload +
                       fifo.p_shm->blksize * fifo.p_shm->rd_index);
    fifo.p_shm->rd_index = (fifo.p_shm->rd_index + 1) % fifo.p_shm->blocks;
    int src_w = input->image.input_w;
    int src_h = input->image.input_h;
    std::unique_lock<std::mutex> mtx(group_list_mtx);
    if (group_list.empty()) {
      auto group = createGroup(input);
      if (group != nullptr) {
        group_list.push_back(group);
      } else {
        sem_t *sem_output = sem_open(input->stamp, O_CREAT);
        sem_post(sem_output);
      }
    } else {
      bool have_same_group = false;
      for (auto &group : group_list) {
        int group_h = group->max_h;
        int group_w = group->max_w;
        if (group_h == src_h && src_w == group_w) {
          std::unique_lock<std::mutex> lock(group->input_list_mtx);
          group->input_list.push_back(input);
          group->active_time = currentMicroseconds();
          group->cv_list_not_empty.notify_one();
          lock.unlock();
          have_same_group = true;
          break;
        }
      }
      if (!have_same_group) {
        if (group_list.size() == 4) {
          RCLCPP_ERROR(rclcpp::get_logger("hobotcv_service"),
                       "hobotcv_service group is full !!");
          sem_t *sem_output = sem_open(input->stamp, O_CREAT);
          sem_post(sem_output);
          sem_post(fifo.sem_mutex);
          sem_post(fifo.sem_full);
        } else {
          auto group = createGroup(input);
          if (group != nullptr) {
            group_list.push_back(group);
          } else {
            sem_t *sem_output = sem_open(input->stamp, O_CREAT);
            sem_post(sem_output);
          }
        }
      }
    }
    mtx.unlock();
  }
}

std::shared_ptr<group_info> hobotcv_service::createGroup(ShmInput_t *input) {
  auto group = std::make_shared<group_info>();
  int i = 0;
  for (i = 4; i < 8; ++i) {  // check group id
    auto flag = groupflag[i];
    if (flag == 0) {
      group->group_id = i;
      groupflag[i] = 1;
      break;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("hobotcv_service"),
              "create group: %d ",
              group->group_id);

  group->max_h = input->image.input_h;
  group->max_w = input->image.input_w;
  group->active_time = currentMicroseconds();

  VPS_GRP_ATTR_S grp_attr;
  grp_attr.maxW = group->max_w;
  grp_attr.maxH = group->max_h;
  grp_attr.frameDepth = 8;
  int ret = HB_VPS_CreateGrp(group->group_id, &grp_attr);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobotcv_service"),
                 "create group: %d failed!!",
                 group->group_id);
    return nullptr;
  }

  //启用channel 0，1，2，5
  groupChn0Init(group->group_id, grp_attr.maxW, grp_attr.maxH);
  groupChn1Init(group->group_id, grp_attr.maxW, grp_attr.maxH);
  groupChn2Init(group->group_id, grp_attr.maxW, grp_attr.maxH);
  groupChn5Init(group->group_id, grp_attr.maxW, grp_attr.maxH);

  ret = HB_VPS_StartGrp(group->group_id);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobotcv_service"), "StartGrp failed!!");
    return nullptr;
  }
  group->input_list.push_back(input);
  group->cv_list_not_empty.notify_one();

  group->work_thread = std::make_shared<std::thread>([this, group] {
    while (1) {
      std::unique_lock<std::mutex> lock(group->input_list_mtx);
      group->cv_list_not_empty.wait(
          lock, [group] { return !group->input_list.empty(); });
      ShmInput_t *input = group->input_list.front();
      group->input_list.pop_front();
      lock.unlock();
      int src_w = input->image.input_w;
      int src_h = input->image.input_h;
      int dst_w = input->image.output_w;
      int dst_h = input->image.output_h;
      int channel_id = -1;
      int enScale = 1;
      if (dst_w > src_w || dst_h > src_h) {  // up scale
        channel_id = 5;
      } else {  // down scale
        if (dst_w > 2048 || dst_h > 1080) {
          channel_id = 2;
        } else if ((dst_w <= 2048 && dst_w > 1280) || dst_h > 720) {
          channel_id = 1;
        } else if (dst_w <= 1280 && dst_h <= 720) {
          channel_id = 0;
        }
      }
      if (dst_w == src_w || dst_h == src_h) {
        enScale = 0;
      }

      auto channel = group->channel_map.find(channel_id);
      if (channel == group->channel_map.end()) {
        setChannelAttr(group->group_id, channel_id, dst_w, dst_h, enScale);
        auto chn = std::make_shared<channel_info>();
        chn->channel_id = channel_id;
        chn->output_w = dst_w;
        chn->output_h = dst_h;
        group->channel_map[channel_id] = chn;
      } else if (channel->second->output_w != dst_w ||
                 channel->second->output_h != dst_h) {
        setChannelAttr(group->group_id, channel_id, dst_w, dst_h, enScale);
        group->channel_map[channel_id]->output_w = dst_w;
        group->channel_map[channel_id]->output_h = dst_h;
      }

      auto it_channel = group->channel_map.find(channel_id);
      if (input->image.rotate != it_channel->second->rotation) {
        it_channel->second->rotation = input->image.rotate;
        setChannelRotate(group->group_id, channel_id, input->image.rotate);
      }

      int ret = HB_VPS_EnableChn(group->group_id, channel_id);
      if (0 != ret) {
        RCLCPP_ERROR(rclcpp::get_logger("hobotcv_service"),
                     "EnableChn failed!!");
        sem_t *sem_output = sem_open(input->stamp, O_CREAT);
        sem_post(sem_output);
        continue;
      }
      ret = sendVpsFrame(input, group->group_id);
      if (ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("hobotcv_service"),
                     "sendVpsFrame to group: %d failed!!",
                     group->group_id);
        sem_t *sem_output = sem_open(input->stamp, O_CREAT);
        sem_post(sem_output);
        continue;
      }

      OutputImage *output = (OutputImage *)shmat(input->output_shmid, NULL, 0);
      if (output == (OutputImage *)-1) {
        RCLCPP_ERROR(rclcpp::get_logger("hobotcv_service"),
                     "shmat output failed shmid: %d!!",
                     input->output_shmid);
        sem_t *sem_output = sem_open(input->stamp, O_CREAT);
        sem_post(sem_output);
        continue;
      }
      hb_vio_buffer_t out_buf;
      ret = getChnFrame(group->group_id, channel_id, out_buf, output);
      if (ret != 0) {
        output->isSuccess = false;
      } else {
        output->isSuccess = true;
      }
      shmdt(output);
      sem_t *sem_output = sem_open(input->stamp, O_CREAT);
      sem_post(sem_output);
      HB_VPS_ReleaseChnFrame(group->group_id, channel_id, &out_buf);
    }
  });

  return group;
}

int hobotcv_service::setChannelAttr(
    int group_id, int chn_id, int width, int height, int enScale) {
  VPS_CHN_ATTR_S chn_attr;
  memset(&chn_attr, 0, sizeof(chn_attr));
  chn_attr.width = width;
  chn_attr.height = height;
  chn_attr.enScale = enScale;
  chn_attr.frameDepth = 8;
  int ret = HB_VPS_SetChnAttr(group_id, chn_id, &chn_attr);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobotcv_service"), "SetChnAttr failed!!");
    return ret;
  }
  return 0;
}

int hobotcv_service::setchannelCrop(VPS_CROP_INFO_S &cropInfo,
                                    int group_id,
                                    int chn_id) {
  int ret = HB_VPS_SetChnCrop(group_id, chn_id, &cropInfo);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobotcv_service"), "SetChnCrop failed!!");
    return ret;
  }
  return 0;
}

int hobotcv_service::setChannelRotate(int group_id, int chn_id, int rotation) {
  ROTATION_E rotate;
  if (rotation == 90) {
    rotate = ROTATION_90;
  } else if (rotation == 180) {
    rotate = ROTATION_180;
  } else if (rotation == 270) {
    rotate = ROTATION_270;
  }
  int ret = HB_VPS_SetChnRotate(group_id, chn_id, rotate);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobotcv_service"),
                 "SetChnRotate failed!!");
    return ret;
  }
  return 0;
}

int hobotcv_service::sendVpsFrame(ShmInput_t *input, int group_id) {
  uint64_t mmz_paddr[2];
  char *mmz_vaddr[2];
  int alloclen = input->image.input_h * input->image.input_w;
  int ret = HB_SYS_Alloc(&mmz_paddr[0], (void **)&mmz_vaddr[0], alloclen);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobotcv_service"),
                 "HB_SYS_Alloc failed!!");
    sem_post(fifo.sem_full);
    sem_post(fifo.sem_mutex);
    return -1;
  }
  alloclen = input->image.input_h * input->image.input_w / 2;
  ret = HB_SYS_Alloc(&mmz_paddr[1], (void **)&mmz_vaddr[1], alloclen);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobotcv_service"),
                 "HB_SYS_Alloc failed!!");
    sem_post(fifo.sem_full);
    sem_post(fifo.sem_mutex);
    return -1;
  }

  char *ydata = &(input->image.inputData[0]);
  char *uvdata = &(input->image.inputData[0]) +
                 input->image.input_w * input->image.input_h;

  memcpy(mmz_vaddr[0], ydata, input->image.input_h * input->image.input_w);
  memcpy(mmz_vaddr[1], uvdata, input->image.input_h * input->image.input_w / 2);
  sem_post(fifo.sem_full);
  sem_post(fifo.sem_mutex);

  hb_vio_buffer_t feedback_buf;
  feedback_buf.img_addr.width = input->image.input_w;
  feedback_buf.img_addr.height = input->image.input_h;
  feedback_buf.img_addr.stride_size = input->image.input_w;

  feedback_buf.img_addr.addr[0] = mmz_vaddr[0];
  feedback_buf.img_addr.addr[1] = mmz_vaddr[1];
  feedback_buf.img_addr.paddr[0] = mmz_paddr[0];
  feedback_buf.img_addr.paddr[1] = mmz_paddr[1];

  ret = HB_VPS_SendFrame(group_id, &feedback_buf, 1000);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobotcv_service"), "SendFrame failed!!");
    return ret;
  }
  HB_SYS_Free(mmz_paddr[0], mmz_vaddr[0]);
  HB_SYS_Free(mmz_paddr[1], mmz_vaddr[1]);
  return 0;
}

int hobotcv_service::getChnFrame(int group_id,
                                 int chn_id,
                                 hb_vio_buffer_t &out_buf,
                                 OutputImage *output) {
  int ret = HB_VPS_GetChnFrame(group_id, chn_id, &out_buf, 2000);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobotcv_service"),
                 "get group: %d chn: %d frame failed!!",
                 group_id,
                 chn_id);
    return -1;
  }
  if (output == (OutputImage *)-1) {
    return -1;
  }
  int stride = out_buf.img_addr.stride_size;
  int width = out_buf.img_addr.width;
  int height = out_buf.img_addr.height;
  output->output_h = height;
  output->output_w = width;

  if (stride == width) {
    memcpy(&output->outputData[0], out_buf.img_addr.addr[0], width * height);
    memcpy(&output->outputData[0] + width * height,
           out_buf.img_addr.addr[1],
           width * height / 2);
  } else {
    int i = 0;
    // jump over stride - width Y
    for (i = 0; i < height; i++) {
      memcpy(&output->outputData[0] + i * width,
             out_buf.img_addr.addr[0] + i * stride,
             width);
    }
    // jump over stride - width UV
    for (i = 0; i < height / 2; i++) {
      memcpy(&output->outputData[0] + width * height + i * width,
             out_buf.img_addr.addr[1] + i * stride,
             width);
    }
  }
  return 0;
}

int hobotcv_service::groupChn0Init(int group_id, int max_w, int max_h) {
  VPS_CHN_ATTR_S chn_attr_max;
  memset(&chn_attr_max, 0, sizeof(chn_attr_max));
  chn_attr_max.width = max_w > 1280 ? 1280 : max_w;
  chn_attr_max.height = max_h > 720 ? 720 : max_h;
  chn_attr_max.enScale = 1;
  chn_attr_max.frameDepth = 8;
  auto ret = HB_VPS_SetChnAttr(group_id, 0, &chn_attr_max);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobotcv_service"),
                 "group: %d Chn0Init failed! chn_width: %d chn_height: %d",
                 group_id,
                 chn_attr_max.width,
                 chn_attr_max.height);
  }
  return 0;
}

int hobotcv_service::groupChn1Init(int group_id, int max_w, int max_h) {
  VPS_CHN_ATTR_S chn_attr_max;
  memset(&chn_attr_max, 0, sizeof(chn_attr_max));
  chn_attr_max.width = max_w > 2048 ? 2048 : max_w;
  chn_attr_max.height = max_h > 1080 ? 1080 : max_h;
  chn_attr_max.enScale = 1;
  chn_attr_max.frameDepth = 8;
  auto ret = HB_VPS_SetChnAttr(group_id, 1, &chn_attr_max);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobotcv_service"),
                 "group: %d Chn1Init failed! chn_width: %d chn_height: %d",
                 group_id,
                 chn_attr_max.width,
                 chn_attr_max.height);
  }

  return 0;
}

int hobotcv_service::groupChn2Init(int group_id, int max_w, int max_h) {
  VPS_CHN_ATTR_S chn_attr_max;
  memset(&chn_attr_max, 0, sizeof(chn_attr_max));
  chn_attr_max.width = max_w > 4096 ? 4096 : max_w;
  chn_attr_max.height = max_h > 2156 ? 2156 : max_h;
  chn_attr_max.enScale = 1;
  chn_attr_max.frameDepth = 8;
  auto ret = HB_VPS_SetChnAttr(group_id, 2, &chn_attr_max);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobotcv_service"),
                 "group: %d Chn2Init failed! chn_width: %d chn_height: %d",
                 group_id,
                 chn_attr_max.width,
                 chn_attr_max.height);
  }
  return 0;
}

int hobotcv_service::groupChn5Init(int group_id, int max_w, int max_h) {
  VPS_CHN_ATTR_S chn_attr_max;
  memset(&chn_attr_max, 0, sizeof(chn_attr_max));
  int max_us_w = max_w * 1.5;
  int max_us_h = max_h * 1.5;
  chn_attr_max.width = max_us_w > 4096 ? 4096 : max_us_w;
  chn_attr_max.height = max_us_h > 2160 ? 2160 : max_us_h;
  chn_attr_max.enScale = 1;
  chn_attr_max.frameDepth = 8;
  auto ret = HB_VPS_SetChnAttr(group_id, 5, &chn_attr_max);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobotcv_service"),
                 "group: %d Chn5Init failed! chn_width: %d chn_height: %d",
                 group_id,
                 chn_attr_max.width,
                 chn_attr_max.height);
  }

  return 0;
}

}  // namespace hobot_cv