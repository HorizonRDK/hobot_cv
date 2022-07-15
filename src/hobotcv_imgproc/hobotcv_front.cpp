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
#include "hobotcv_imgproc/hobotcv_front.h"

#include <errno.h>
#include <string.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "utils.h"

namespace hobot_cv {

hobotcv_front::hobotcv_front() {
  roi.cropEnable = 0;
  roi.x = 0;
  roi.y = 0;
  roi.width = 0;
  roi.height = 0;
}

hobotcv_front::~hobotcv_front() {}

int hobotcv_front::shmfifoInit() {
  memset(&fifo, 0, sizeof(hobot_cv::shmfifo_t));
  // Init sem
  int size = sizeof(hobot_cv::shmhead_t) + INPUT_SHM_SIZE * sizeof(ShmInput_t);
  int shmid = shmget((key_t)1234, 0, 0);
  if (shmid < 0) {
    RCLCPP_INFO(rclcpp::get_logger("hobot_cv"), "create input shared memory!!");
    //创建新的共享内存区,返回共享内存标识符
    fifo.shmid = shmget((key_t)1234, size, IPC_CREAT | 0666);
    if (fifo.shmid == -1) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "shmfifo shmget failed!!");
      return -1;
    }
    // fino->p_shm 指向共享内存头部指针
    fifo.p_shm = (hobot_cv::shmhead_t *)shmat(fifo.shmid, NULL, 0);
    //文档中有说明 "error (void *) -1 is returned"
    if (fifo.p_shm == (hobot_cv::shmhead_t *)-1) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "shmfifo shmat failed!!");
      return -1;
    }
    //+1指针偏移,得到有效负载起始地址
    fifo.p_payload = (char *)(fifo.p_shm + 1);

    fifo.p_shm->blksize = sizeof(ShmInput_t);
    fifo.p_shm->blocks = INPUT_SHM_SIZE;
    fifo.p_shm->rd_index = 0;
    fifo.p_shm->wr_index = 0;
    fifo.p_shm->service_launch = false;

    //创建4个信号量
    fifo.sem_mutex = sem_open("/sem_input", O_CREAT, 0666, 1);  // 互斥信号量
    fifo.sem_full =
        sem_open("/sem_input_full", O_CREAT, 0666, INPUT_SHM_SIZE);  //满信号量
    fifo.sem_empty =
        sem_open("/sem_input_empty", O_CREAT, 0666, 0);  // 空信号量
    fifo.sem_output =
        sem_open("/sem_output_key", O_CREAT, 0666, 1);  // output互斥
  } else {
    fifo.shmid = shmid;
    //连接 shm
    fifo.p_shm = (hobot_cv::shmhead_t *)shmat(fifo.shmid, NULL, 0);
    if (fifo.p_shm == (hobot_cv::shmhead_t *)-1) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "shmfifo shmat failed!!");
      return -1;
    }
    //+1指针偏移,得到有效负载起始地址
    fifo.p_payload = (char *)(fifo.p_shm + 1);
    fifo.sem_mutex = sem_open("/sem_input", O_CREAT);
    fifo.sem_full = sem_open("/sem_input_full", O_CREAT);
    fifo.sem_empty = sem_open("/sem_input_empty", O_CREAT);
    fifo.sem_output = sem_open("/sem_output_key", O_CREAT);
  }
  return 0;
}

int hobotcv_front::prepareResizeParam(int src_width,
                                      int src_height,
                                      int dst_width,
                                      int dst_height) {
  int resize_src_width = roi.cropEnable ? roi.width : src_width;
  int resize_src_height = roi.cropEnable ? roi.height : src_height;
  if (dst_width % 4 != 0 || dst_height % 2 != 0 || dst_height > 2160 ||
      dst_width > 4096 || dst_height < 32 || dst_width < 32) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "The dst width should be a multiple of 4 and the "
                 "height should be even! The output resolution ranges from 32 "
                 "x 32 to 4096 x 2160 !");
    return -1;
  }
  if (resize_src_height < 32 || resize_src_width < 32 ||
      resize_src_height > 2160 || resize_src_width > 4096) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "The input resolution ranges from 32 x 32 to 4096 x 4096");
  }
  if (dst_height > resize_src_height * 1.5 ||
      dst_width > resize_src_width * 1.5) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "Max 1.5x upscale is supported");
    return -1;
  }
  if (dst_width < resize_src_width / 8 || dst_height < resize_src_width / 8) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "Max 1/8 upscale is supported");
    return -1;
  }
  this->src_h = src_height;
  this->src_w = src_width;
  this->dst_h = dst_height;
  this->dst_w = dst_width;
  return 0;
}

int hobotcv_front::prepareRotateParam(int rotation) {
  switch (rotation) {
    case 0:
      rotate = 0;
      return 1;
      break;
    case 1:
      rotate = 90;
      break;
    case 2:
      rotate = 180;
      break;
    case 3:
      rotate = 270;
      break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                   "hobot_cv only supports 90、180、270 rotation!");
      return -1;
      break;
  }
  return 0;
}

int hobotcv_front::prepareCropRoi(int &src_height,
                                  int &src_width,
                                  int &dst_width,
                                  int &dst_height,
                                  const cv::Range &rowRange,
                                  const cv::Range &colRange) {
  if (colRange.end - colRange.start <= 0 ||
      rowRange.end - rowRange.start <= 0) {
    roi.cropEnable = 0;
  } else {
    if (rowRange.end <= rowRange.start || colRange.end <= colRange.start ||
        rowRange.start < 0 || colRange.start < 0) {
      RCLCPP_ERROR(
          rclcpp::get_logger("hobot_cv"),
          "Invalid Range data! The end data must be greater than the "
          "start data and the starting value cannot be less than zero!");
      return -1;
    }
    if (rowRange.end > src_height || colRange.end > src_width) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                   "Invalid Range data! rowRange should in [0, %d) and "
                   "colRange should in [0, %d)",
                   src_height,
                   src_width);
      return -1;
    }
    if (dst_height > (rowRange.end - rowRange.start) * 1.5 ||
        dst_width > (colRange.end - colRange.start) * 1.5) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "dst > cropArea * 1.5");
      return -1;
    }
    if (dst_width < (colRange.end - colRange.start) / 8 ||
        dst_height < (rowRange.end - rowRange.start) / 8) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "dst < cropArea / 8");
      return -1;
    }
    roi.cropEnable = 1;
    roi.x = colRange.start;
    roi.y = rowRange.start;
    roi.width = colRange.end - colRange.start;
    roi.height = rowRange.end - rowRange.start;
  }
  return 0;
}

int hobotcv_front::createInputImage(const cv::Mat &src) {
  sem_wait(fifo.sem_full);
  sem_wait(fifo.sem_mutex);
  ShmInput_t *input = (ShmInput_t *)(fifo.p_payload + fifo.p_shm->wr_index *
                                                          fifo.p_shm->blksize);
  memset(input, 0, sizeof(ShmInput_t));
  input->image.input_h = roi.cropEnable == 1 ? roi.height : src_h;
  input->image.input_w = roi.cropEnable == 1 ? roi.width : src_w;
  input->image.output_h = dst_h;
  input->image.output_w = dst_w;
  input->image.rotate = rotate;
  std::string str_stamp;
  auto stamp = currentMicroseconds();
  std::stringstream ss;
  ss << "/iutput_" << stamp;
  ss >> str_stamp;
  strcpy(input->stamp, str_stamp.c_str());

  sem_wait(fifo.sem_output);
  for (size_t i = 0; i < 20; i++) {
    if (fifo.p_shm->output_shmKey[i] == 0) {
      output_shmkeyIndex = i;
      int key = 1235 + i;
      //创建输出图片共享内存,返回共享内存标识符
      input->output_shmid =
          shmget((key_t)key, sizeof(OutputImage), IPC_CREAT | 0666);
      if (input->output_shmid == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "output shmget failed!!");
        sem_post(fifo.sem_output);
        sem_post(fifo.sem_mutex);
        return -1;
      }
      output_shmid = input->output_shmid;
      fifo.p_shm->output_shmKey[i] = 1;
      break;
    }
  }
  sem_post(fifo.sem_output);
  if (output_shmkeyIndex == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "get output shmkey failed!!");
    sem_post(fifo.sem_mutex);
    return -1;
  }

  // 映射到输出图片的共享内存
  output = (OutputImage *)shmat(input->output_shmid, NULL, 0);
  if (output == (OutputImage *)-1) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "output shmat failed!!");
    sem_post(fifo.sem_mutex);
    return -1;
  }
  memset(output, 0, sizeof(OutputImage));
  output->isSuccess = false;

  sem_output = sem_open(str_stamp.c_str(), O_CREAT, 0666, 0);  // 输出图片信号量

  if (roi.cropEnable == 1) {
    auto srcdata = reinterpret_cast<const uint8_t *>(src.data);
    auto dstdata = &(input->image.inputData[0]);
    // copy y
    for (int h = 0; h < roi.height; ++h) {
      auto *raw = dstdata + h * roi.width;
      auto *src = srcdata + (h + roi.y) * src_w + roi.x;
      memcpy(raw, src, roi.width);
    }

    // copy uv
    auto uv_data = srcdata + src_h * src_w;
    auto dstuvdata = dstdata + roi.height * roi.width;
    for (int32_t h = 0; h < roi.height / 2; ++h) {
      auto *raw = dstuvdata + h * roi.width;
      auto *src = uv_data + (h + (roi.y / 2)) * src_w + roi.x;
      memcpy(raw, src, roi.width);
    }
  } else {
    size_t size = input->image.input_h * input->image.input_w * 3 / 2;
    memcpy(&(input->image.inputData[0]), src.data, size);
  }
  fifo.p_shm->wr_index = (fifo.p_shm->wr_index + 1) % fifo.p_shm->blocks;

  sem_post(fifo.sem_empty);
  sem_post(fifo.sem_mutex);
  return 0;
}

int hobotcv_front::getOutputImage(cv::Mat &dst) {
  sem_wait(sem_output);
  if (output->isSuccess == false) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "hobotcv_service get frame failed!");
    shmdt(fifo.p_shm);
    shmdt(output);
    shmctl(output_shmid, IPC_RMID, NULL);
    sem_destroy(sem_output);
    sem_wait(fifo.sem_output);
    fifo.p_shm->output_shmKey[output_shmkeyIndex] = 0;
    sem_post(fifo.sem_output);
    return -1;
  }

  dst = cv::Mat(output->output_h * 3 / 2, output->output_w, CV_8UC1);
  memcpy(dst.data,
         output->outputData,
         output->output_h * output->output_w * 3 / 2);

  shmdt(output);
  shmctl(output_shmid, IPC_RMID, NULL);
  sem_destroy(sem_output);
  sem_wait(fifo.sem_output);
  fifo.p_shm->output_shmKey[output_shmkeyIndex] = 0;
  sem_post(fifo.sem_output);
  shmdt(fifo.p_shm);
  return 0;
}

int hobotcv_front::hobotcv_bpu_resize(const cv::Mat &src,
                                      int src_h,
                                      int src_w,
                                      cv::Mat &dst,
                                      int dst_h,
                                      int dst_w,
                                      const cv::Range &rowRange,
                                      const cv::Range &colRange) {
  auto range_h = rowRange.end - rowRange.start;
  auto range_w = colRange.end - colRange.start;
  if (colRange.start < 0 || rowRange.start < 0 ||
      colRange.start + range_w > src_w || rowRange.start + range_h > src_h) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "Invalid Range data! RowRnage should be [0, %d) and "
                 "colRange should be [0, %d)",
                 src_h,
                 src_w);
    return -1;
  }
  dst = cv::Mat(dst_h * 3 / 2, dst_w, CV_8UC1);
  hbDNNRoi roi;
  roi.left = colRange.start;
  roi.top = rowRange.start;
  if (rowRange.end <= rowRange.start || colRange.end <= colRange.start) {
    roi.right = 0;
    roi.bottom = 0;
  } else {
    roi.right = colRange.end - 1;
    roi.bottom = rowRange.end - 1;
  }
  if (range_h == dst_h && range_w == dst_w) {  // crop only
    dst = cv::Mat(dst_h * 3 / 2, dst_w, CV_8UC1);
    auto srcdata = reinterpret_cast<const uint8_t *>(src.data);
    auto dstdata = dst.data;
    // copy y
    for (int h = 0; h < dst_h; ++h) {
      auto *raw = dstdata + h * dst_w;
      auto *src = srcdata + (h + roi.top) * src_w + roi.left;
      memcpy(raw, src, dst_w);
    }

    // copy uv
    auto uv_data = srcdata + src_h * src_w;
    auto dstuvdata = dstdata + dst_h * dst_w;
    for (int32_t h = 0; h < dst_h / 2; ++h) {
      auto *raw = dstuvdata + h * dst_w;
      auto *src = uv_data + (h + (roi.top / 2)) * src_w + roi.left;
      memcpy(raw, src, dst_w);
    }
    return 0;
  }

  hbDNNTensor input_tensor;
  prepare_nv12_tensor_without_padding(src.data, src_h, src_w, &input_tensor);
  // Prepare output tensor
  hbDNNTensor output_tensor;
  prepare_nv12_tensor_without_padding(dst_h, dst_w, &output_tensor);
  // resize
  hbDNNResizeCtrlParam ctrl = {HB_BPU_CORE_0, 0, HB_DNN_RESIZE_TYPE_BILINEAR};
  hbDNNTaskHandle_t task_handle = nullptr;

  int ret = 0;
  if (roi.right == 0 && roi.bottom == 0) {
    ret = hbDNNResize(
        &task_handle, &output_tensor, &input_tensor, nullptr, &ctrl);
  } else {
    ret = hbDNNResize(&task_handle, &output_tensor, &input_tensor, &roi, &ctrl);
  }

  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "hbDNNResize failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return -1;
  }
  ret = hbDNNWaitTaskDone(task_handle, 0);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "hbDNNWaitTaskDone failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return -1;
  }
  hbDNNReleaseTask(task_handle);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "release task failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return -1;
  }

  size_t size = dst_h * dst_w * 3 / 2;
  memcpy(dst.data, output_tensor.sysMem[0].virAddr, size);
  hbSysFreeMem(&(input_tensor.sysMem[0]));
  hbSysFreeMem(&(output_tensor.sysMem[0]));
  return 0;
}

}  // namespace hobot_cv
