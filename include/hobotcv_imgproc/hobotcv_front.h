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

#include <fcntl.h>
#include <semaphore.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <memory>
#include <string>
#include <vector>

#include "dnn/hb_dnn.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#define INPUT_SEM "input_image"
#define IMAGE_MAX_LENGTH (4096 * 2160 * 3 / 2)
#define INPUT_SHM_SIZE (4)

namespace hobot_cv {

// input共享内存头部信息
typedef struct shmhead {
  uint32_t blksize;       //每一块大小
  uint32_t blocks;        //共享内存块数
  uint32_t rd_index;      //读索引
  uint32_t wr_index;      //写索引
  bool service_launch;    // hobotcv_service是否已经启动
  int output_shmKey[20];  //用于获取输出图片共享内存key
} shmhead_t;

// input共享内存结构信息
typedef struct shminfo {
  hobot_cv::shmhead_t *p_shm;  //头部指针信息
  char *p_payload;             //有效负载起始地址
  int shmid;                   //共享内存id
  sem_t *sem_mutex;  //用来互斥量的信号量,生产者消费者用来占用仓库的信号量
  sem_t *sem_full;   //仓库已满的信号量
  sem_t *sem_empty;  //仓库已空的信号量
  sem_t *sem_output;  //用于不同调用进程在返回图片时更改output_shmKey
} shmfifo_t;

typedef struct HOBOT_CV_CROP_RECT {
  int cropEnable;  // crop使能
  uint16_t x;
  uint16_t y;
  uint16_t width;
  uint16_t height;
} CropRect;

typedef struct HOBOT_CV_INPUT_IMAGE {
  char inputData[IMAGE_MAX_LENGTH];  //输入图片数据，nv12格式
  int input_w;                       //输入图片宽度
  int input_h;                       //输入图片高度
  int output_w;                      //输出图片宽度
  int output_h;                      //输出图片高度
  int rotate;  //输出图片旋转角度，0，90，180，270
  // CropRect roi;  // crop属性
  // int pymEnable;                 //金字塔处理使能 0/1
  // PyramidAttr pymattr;           //金字塔处理配置参数
} InputImage_t;

typedef struct HOBOT_CV_INPUT {
  InputImage_t image;
  char stamp[20];    //用于输出处理后信号量
  int output_shmid;  // service处理后的图片数据存放的共享内存id
} ShmInput_t;

typedef struct HBOT_CV_OUTPUT_IMAGE {
  bool isSuccess;
  char outputData[IMAGE_MAX_LENGTH];
  int output_w;
  int output_h;
} OutputImage;

class hobotcv_front {
 public:
  hobotcv_front();
  ~hobotcv_front();

  int shmfifoInit();

  int hobotcv_bpu_resize(const cv::Mat &src,
                         int src_h,
                         int src_w,
                         cv::Mat &dst,
                         int dst_h,
                         int dst_w,
                         const cv::Range &rowRange,
                         const cv::Range &colRange);

  int prepareResizeParam(int src_width,
                         int src_height,
                         int dst_width,
                         int dst_height);

  int prepareRotateParam(int rotation);

  int prepareCropRoi(int &src_height,
                     int &src_width,
                     int &dst_width,
                     int &dst_height,
                     const cv::Range &rowRange,
                     const cv::Range &colRange);

  int createInputImage(const cv::Mat &src);
  int getOutputImage(cv::Mat &dst);

  bool getServiceLaunched() { return fifo.p_shm->service_launch; }

  void setServiceLaunched(bool launched) {
    fifo.p_shm->service_launch = launched;
  }

 public:
  int src_w;
  int src_h;
  int dst_w;
  int dst_h;
  int rotate = 0;
  CropRect roi;

 private:
  shmfifo_t fifo;
  OutputImage *output;  //映射到输出图片的共享内存
  sem_t *sem_output;
  int output_shmid;
  int output_shmkeyIndex = -1;
};

}  // namespace hobot_cv

#endif  // HOBOTCV_FRONT_H
