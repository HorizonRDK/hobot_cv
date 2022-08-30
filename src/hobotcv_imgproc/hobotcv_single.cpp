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
#include "hobotcv_imgproc/hobotcv_single.h"

namespace hobot_cv {
int hobotcv_single::shmfifoInit() {
  memset(&fifo, 0, sizeof(shmfifo_t));
  // Init sem
  //创建共享内存互斥信号
  sem_t *shm_sem = sem_open("/sem_shm", O_CREAT, 0666, 1);
  sem_wait(shm_sem);
  int shmid = shmget((key_t)1234, 0, 0);
  if (shmid < 0) {
    size_t size = sizeof(Group_info_t) * HOBOTCV_GROUP_SIZE;
    //创建新的共享内存区,返回共享内存标识符
    fifo.shmid = shmget((key_t)1234, size, IPC_CREAT | 0666);
    if (fifo.shmid == -1) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "shmfifo shmget failed!!");
      sem_post(shm_sem);
      sem_close(shm_sem);
      return -1;
    }

    fifo.groups = shmat(fifo.shmid, NULL, 0);
    if (fifo.groups == (void *)-1) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "shmfifo shmat failed!!");
      sem_post(shm_sem);
      sem_close(shm_sem);
      return -1;
    }
    memset(fifo.groups, 0, size);

    //创建信号量
    fifo.sem_groups = sem_open("/sem_allgroup", O_CREAT, 0666, 1);
    fifo.sem_group4 = sem_open("/sem_group4", O_CREAT, 0666, 1);
    fifo.sem_group5 = sem_open("/sem_group5", O_CREAT, 0666, 1);
    fifo.sem_group6 = sem_open("/sem_group6", O_CREAT, 0666, 1);
    fifo.sem_group7 = sem_open("/sem_group7", O_CREAT, 0666, 1);
  } else {
    fifo.shmid = shmid;
    //连接 shm
    fifo.groups = shmat(fifo.shmid, NULL, 0);
    if (fifo.groups == (void *)-1) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "shmfifo shmat failed!!");
      sem_post(shm_sem);
      sem_close(shm_sem);
      return -1;
    }
    //打开信号量
    fifo.sem_groups = sem_open("/sem_allgroup", O_CREAT);
    fifo.sem_group4 = sem_open("/sem_group4", O_CREAT);
    fifo.sem_group5 = sem_open("/sem_group5", O_CREAT);
    fifo.sem_group6 = sem_open("/sem_group6", O_CREAT);
    fifo.sem_group7 = sem_open("/sem_group7", O_CREAT);
  }
  sem_post(shm_sem);
  sem_close(shm_sem);
  return 0;
}

void hobotcv_single::Hobotcv_AddGroup(int group_id, hobotcv_sys_mem &sys_mem) {
  std::unique_lock<std::mutex> lk(group_map_mtx);
  group_map[group_id] = sys_mem;
  lk.unlock();
}

hobotcv_sys_mem &hobotcv_single::GetGroupSysmem(int group_id) {
  return group_map[group_id];
}
}  // namespace hobot_cv
