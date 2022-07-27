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

  pym_param.pymEnable = 0;
}

hobotcv_front::~hobotcv_front() {}

int hobotcv_front::shmfifoInit() {
  memset(&fifo, 0, sizeof(hobot_cv::shmfifo_t));
  // Init sem
  int shmid = shmget((key_t)1234, 0, 0);
  if (shmid < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "launch hobot_service first!");
    return -1;
  } else {
    fifo.shmid = shmid;
    //连接 shm
    fifo.p_shm = (hobot_cv::shmhead_t *)shmat(fifo.shmid, NULL, 0);
    if (fifo.p_shm == (hobot_cv::shmhead_t *)-1) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "shmfifo shmat failed!!");
      return -1;
    }
    //指针偏移,得到有效负载起始地址
    fifo.p_InputPayload = (char *)(fifo.p_shm + 1);
    fifo.p_OutputPayload =
        fifo.p_InputPayload + INPUT_SHM_SIZE * sizeof(ShmInput_t);
    fifo.p_PymOutPutPayload =
        fifo.p_OutputPayload + OUTPUT_SHM_SIZE * sizeof(OutputImage);
    fifo.sem_mutex = sem_open("/sem_input", O_CREAT);
    fifo.sem_full = sem_open("/sem_input_full", O_CREAT);
    fifo.sem_empty = sem_open("/sem_input_empty", O_CREAT);
    fifo.sem_output = sem_open("/sem_output_key", O_CREAT);
    fifo.sem_pymout = sem_open("/sem_pym_out", O_CREAT);
  }
  return 0;
}

int hobotcv_front::prepareResizeParam(int src_width,
                                      int src_height,
                                      int dst_width,
                                      int dst_height,
                                      bool printLog) {
  int resize_src_width = roi.cropEnable ? roi.width : src_width;
  int resize_src_height = roi.cropEnable ? roi.height : src_height;
  if (dst_width % 16 != 0 || dst_height % 2 != 0 || dst_height > 2160 ||
      dst_width > 4096 || dst_height < 32 || dst_width < 32) {
    if (printLog) {
      RCLCPP_ERROR(
          rclcpp::get_logger("hobot_cv"),
          "The dst width should be a multiple of 16 and the "
          "height should be even! The output resolution ranges from 32 "
          "x 32 to 4096 x 2160 !");
    }
    return -1;
  }
  if (resize_src_width % 16 != 0 || resize_src_height % 2 != 0 ||
      resize_src_height < 32 || resize_src_width < 32 ||
      resize_src_height > 2160 || resize_src_width > 4096) {
    if (printLog) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                   "The input resolution ranges from 32 x 32 to 4096 x 4096");
    }
    return -1;
  }
  if (dst_height > resize_src_height * 1.5 ||
      dst_width > resize_src_width * 1.5) {
    if (printLog) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                   "Max 1.5x upscale is supported");
    }
    return -1;
  }
  if (dst_width < resize_src_width / 8 || dst_height < resize_src_width / 8) {
    if (printLog) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                   "Max 1/8 upscale is supported");
    }
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

int hobotcv_front::prepareCropRoi(int src_height,
                                  int src_width,
                                  int dst_width,
                                  int dst_height,
                                  const cv::Range &rowRange,
                                  const cv::Range &colRange,
                                  bool printLog) {
  if (colRange.end - colRange.start <= 0 ||
      rowRange.end - rowRange.start <= 0) {
    roi.cropEnable = 0;
  } else {
    if (rowRange.end <= rowRange.start || colRange.end <= colRange.start ||
        rowRange.start < 0 || colRange.start < 0) {
      if (printLog) {
        RCLCPP_ERROR(
            rclcpp::get_logger("hobot_cv"),
            "Invalid Range data! The end data must be bigger than the "
            "start data and the starting value cannot be less than zero!");
      }
      return -1;
    }
    if (rowRange.end > src_height || colRange.end > src_width) {
      if (printLog) {
        RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                     "Invalid Range data! rowRange should in [0, %d) and "
                     "colRange should in [0, %d)",
                     src_height,
                     src_width);
      }
      return -1;
    }
    if (dst_height > (rowRange.end - rowRange.start) * 1.5 ||
        dst_width > (colRange.end - colRange.start) * 1.5) {
      if (printLog) {
        RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "dst > cropArea * 1.5");
      }
      return -1;
    }
    if (dst_width < (colRange.end - colRange.start) / 8 ||
        dst_height < (rowRange.end - rowRange.start) / 8) {
      if (printLog) {
        RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "dst < cropArea / 8");
      }
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

int hobotcv_front::preparePymraid(int src_h,
                                  int src_w,
                                  const PyramidAttr &attr) {
  if (src_h > 4096 || src_w > 4096 || src_h < 64 || src_w < 64) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "The src resolution ranges from 64 x 64 to 4096 x 4096 !");
    return -1;
  }
  memcpy(&pym_param.attr, &attr, sizeof(PyramidAttr));
  pym_param.pymEnable = 1;
  this->src_h = src_h;
  this->src_w = src_w;
  return 0;
}

int hobotcv_front::createInputImage(const cv::Mat &src) {
  sem_wait(fifo.sem_full);
  sem_wait(fifo.sem_mutex);
  ShmInput_t *input =
      (ShmInput_t *)(fifo.p_InputPayload +
                     fifo.p_shm->wr_index * fifo.p_shm->blksize);
  memset(input, 0, sizeof(ShmInput_t));
  input->image.input_h = roi.cropEnable == 1 ? roi.height : src_h;
  input->image.input_w = roi.cropEnable == 1 ? roi.width : src_w;
  input->image.output_h = dst_h;
  input->image.output_w = dst_w;
  input->image.rotate = rotate;
  memcpy(&input->image.pymparam, &pym_param, sizeof(PyramidParam));
  auto stamp = currentMicroseconds();
  std::stringstream ss;
  ss << "/output_" << stamp;
  ss >> str_stamp;
  strcpy(input->stamp, str_stamp.c_str());

  if (pym_param.pymEnable == 1) {
    sem_wait(fifo.sem_pymout);
    for (size_t i = 0; i < OUTPUT_PYM_SHM_SIZE; i++) {
      if (fifo.p_shm->output_pym_shmIndex[i] == 0) {
        output_pym_Index = i;
        input->pym_out_index = i;
        fifo.p_shm->output_pym_shmIndex[i] = 1;
        break;
      }
    }
    sem_post(fifo.sem_pymout);
    if (output_pym_Index == -1) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                   "get pymout shmkey failed!!");
      sem_post(fifo.sem_mutex);
      shmdt(fifo.p_shm);
      sem_close(fifo.sem_empty);
      sem_close(fifo.sem_full);
      sem_close(fifo.sem_mutex);
      sem_close(fifo.sem_output);
      sem_close(fifo.sem_pymout);
      return -1;
    }

    hobotcv_pymOutput =
        (OutputPyramid *)(fifo.p_PymOutPutPayload +
                          sizeof(OutputPyramid) * output_pym_Index);
    memset(hobotcv_pymOutput, 0, sizeof(OutputPyramid));
    hobotcv_pymOutput->isSuccess = false;

    hobotcv_sem_output =
        sem_open(str_stamp.c_str(), O_CREAT, 0666, 0);  // 输出图片信号量
    size_t size = input->image.input_h * input->image.input_w * 3 / 2;
    memcpy(&(input->image.inputData[0]), src.data, size);
    fifo.p_shm->wr_index =
        (fifo.p_shm->wr_index + 1) % fifo.p_shm->input_blocks;
    sem_post(fifo.sem_empty);
    sem_post(fifo.sem_mutex);
    return 0;
  }

  sem_wait(fifo.sem_output);
  for (size_t i = 0; i < OUTPUT_SHM_SIZE; i++) {
    if (fifo.p_shm->output_shmIndex[i] == 0) {
      output_shm_Index = i;
      input->output_shm_index = i;
      fifo.p_shm->output_shmIndex[i] = 1;
      break;
    }
  }
  sem_post(fifo.sem_output);
  if (output_shm_Index == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "get output shmkey failed!!");
    sem_post(fifo.sem_mutex);
    shmdt(fifo.p_shm);
    sem_close(fifo.sem_empty);
    sem_close(fifo.sem_full);
    sem_close(fifo.sem_mutex);
    sem_close(fifo.sem_output);
    sem_close(fifo.sem_pymout);
    return -1;
  }

  // 映射到输出图片的共享内存
  hobotcv_output = (OutputImage *)(fifo.p_OutputPayload +
                                   sizeof(OutputImage) * output_shm_Index);
  memset(hobotcv_output, 0, sizeof(OutputImage));
  hobotcv_output->isSuccess = false;

  hobotcv_sem_output =
      sem_open(str_stamp.c_str(), O_CREAT, 0666, 0);  // 输出图片信号量

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
  fifo.p_shm->wr_index = (fifo.p_shm->wr_index + 1) % fifo.p_shm->input_blocks;
  sem_post(fifo.sem_empty);
  sem_post(fifo.sem_mutex);
  return 0;
}

int hobotcv_front::getOutputImage(cv::Mat &dst) {
  sem_wait(hobotcv_sem_output);

  if (hobotcv_output->isSuccess == false) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "hobotcv_service get frame failed!");

    sem_close(fifo.sem_empty);
    sem_close(fifo.sem_full);
    sem_close(fifo.sem_mutex);
    sem_close(hobotcv_sem_output);
    sem_unlink(str_stamp.c_str());
    sem_wait(fifo.sem_output);
    fifo.p_shm->output_shmIndex[output_shm_Index] = 0;
    sem_post(fifo.sem_output);
    sem_close(fifo.sem_output);
    sem_close(fifo.sem_pymout);
    shmdt(fifo.p_shm);
    return -1;
  }

  dst = cv::Mat(
      hobotcv_output->output_h * 3 / 2, hobotcv_output->output_w, CV_8UC1);
  memcpy(dst.data,
         hobotcv_output->outputData,
         hobotcv_output->output_h * hobotcv_output->output_w * 3 / 2);

  sem_close(fifo.sem_empty);
  sem_close(fifo.sem_full);
  sem_close(fifo.sem_mutex);
  sem_close(hobotcv_sem_output);
  sem_unlink(str_stamp.c_str());
  sem_wait(fifo.sem_output);
  fifo.p_shm->output_shmIndex[output_shm_Index] = 0;
  sem_post(fifo.sem_output);
  sem_close(fifo.sem_output);
  sem_close(fifo.sem_pymout);
  shmdt(fifo.p_shm);
  return 0;
}

int hobotcv_front::getPyramidOutputImage(OutputPyramid *output) {
  sem_wait(hobotcv_sem_output);

  if (hobotcv_pymOutput->isSuccess == false) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "hobotcv_service get pym frame failed!");

    sem_close(fifo.sem_empty);
    sem_close(fifo.sem_full);
    sem_close(fifo.sem_mutex);
    sem_close(hobotcv_sem_output);
    sem_unlink(str_stamp.c_str());
    sem_wait(fifo.sem_pymout);
    fifo.p_shm->output_pym_shmIndex[0] = 0;
    sem_post(fifo.sem_pymout);
    sem_close(fifo.sem_pymout);
    sem_close(fifo.sem_output);
    shmdt(fifo.p_shm);
    return -1;
  }
  output->isSuccess = hobotcv_pymOutput->isSuccess;
  int ds_base_index = 0;
  for (size_t i = 0; i < pym_param.attr.ds_layer_en; i++) {
    if (i % 4 == 0) {
      int width = hobotcv_pymOutput->pym_ds[ds_base_index].width;
      int height = hobotcv_pymOutput->pym_ds[ds_base_index].height;
      output->pym_ds[ds_base_index].width = width;
      output->pym_ds[ds_base_index].height = height;
      memcpy(&(output->pym_ds[ds_base_index].img[0]),
             &(hobotcv_pymOutput->pym_ds[ds_base_index].img[0]),
             width * height * 3 / 2);
      ds_base_index++;
    }
  }

  sem_close(fifo.sem_empty);
  sem_close(fifo.sem_full);
  sem_close(fifo.sem_mutex);
  sem_close(hobotcv_sem_output);
  sem_unlink(str_stamp.c_str());
  sem_wait(fifo.sem_pymout);
  fifo.p_shm->output_pym_shmIndex[0] = 0;
  sem_post(fifo.sem_pymout);
  sem_close(fifo.sem_pymout);
  sem_close(fifo.sem_output);
  shmdt(fifo.p_shm);
  return 0;
}

}  // namespace hobot_cv
