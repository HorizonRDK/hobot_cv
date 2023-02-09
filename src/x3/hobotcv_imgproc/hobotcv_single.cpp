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
  // Init sem
  //创建共享内存互斥信号
  sem_t *shm_sem = sem_open("/sem_shm", O_CREAT, 0666, 1);
  sem_wait(shm_sem);

  //系统第一次创建共享内存，状态莫名为dest
  key_t key = ftok("/root", 0x6666);
  int shmid = shmget(key, 0, 0);
  if (shmid < 0) {
    shmget(key, 10, IPC_CREAT | 0666);
  }

  shmid = shmget((key_t)1234, 0, 0);
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
  }
  sem_post(shm_sem);
  sem_close(shm_sem);
  return 0;
}

void hobotcv_single::HobotcvAddGroup(int group_id, hobotcv_sys_mem &sys_mem) {
  if (group_map.find(group_id) != group_map.end()) {
    //释放原有内存，重新添加新内存
    HB_SYS_Free(group_map[group_id].mmz_paddr[0],
                group_map[group_id].mmz_vaddr[0]);
    HB_SYS_Free(group_map[group_id].mmz_paddr[1],
                group_map[group_id].mmz_vaddr[1]);
  }
  group_map[group_id] = sys_mem;
}

hobotcv_sys_mem &hobotcv_single::GetGroupSysmem(int group_id) {
  return group_map[group_id];
}

void hobotcv_single::AddGroupTimeOut(int group_id) {
  group_map[group_id].timeout_num++;
}

void hobotcv_single::ResetGroupTimeOutNum(int group_id) {
  group_map[group_id].timeout_num = 0;
}

int hobotcv_single::GetGroupTimeOut(int group_id) {
  return group_map[group_id].timeout_num;
}

}  // namespace hobot_cv
