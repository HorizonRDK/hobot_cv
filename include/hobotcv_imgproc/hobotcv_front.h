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
#include "hobotcv_imgproc/hobotcv_imgproc.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#define IMAGE_MAX_LENGTH (4096 * 2160 * 3 / 2)
#define INPUT_SHM_SIZE (4)
#define OUTPUT_SHM_SIZE (4)
#define OUTPUT_PYM_SHM_SIZE (1)

namespace hobot_cv {

// input共享内存头部信息
typedef struct shmhead {
  uint32_t blksize;                      //每一块大小
  uint32_t input_blocks;                 //共享内存块数
  uint32_t rd_index;                     //读索引
  uint32_t wr_index;                     //写索引
  bool service_launch;                   // hobotcv_service是否已经启动
  int output_shmIndex[OUTPUT_SHM_SIZE];  //用于获取输出图片共享内存索引
  int output_pym_shmIndex
      [OUTPUT_PYM_SHM_SIZE];  //用于获取金字塔输出图片共享内存索引
} shmhead_t;

// input共享内存结构信息
typedef struct shminfo {
  hobot_cv::shmhead_t *p_shm;  //头部指针信息
  char *p_InputPayload;        // Input有效负载起始地址
  char *p_OutputPayload;       // Output有效负载起始地址
  char *p_PymOutPutPayload;    // 金字塔输出图片有效负载起始地址
  int shmid;                   //共享内存id
  sem_t *sem_mutex;  //用来互斥量的信号量,生产者消费者用来占用仓库的信号量
  sem_t *sem_full;   //仓库已满的信号量
  sem_t *sem_empty;  //仓库已空的信号量
  sem_t *sem_output;  //用于不同调用进程在返回图片时更改output_shmIndex
  sem_t *sem_pymout;
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

typedef struct HOBOT_CV_INPUT_IMAGE {
  char inputData[IMAGE_MAX_LENGTH];  //输入图片数据，nv12格式
  int input_w;                       //输入图片宽度
  int input_h;                       //输入图片高度
  int output_w;                      //输出图片宽度
  int output_h;                      //输出图片高度
  int rotate;             //输出图片旋转角度，0，90，180，270
  PyramidParam pymparam;  //金字塔处理配置参数
} InputImage_t;

typedef struct HOBOT_CV_INPUT {
  InputImage_t image;
  char stamp[20];        //用于输出处理后信号量
  int output_shm_index;  // service处理后的图片数据存放的共享内存索引
  int pym_out_index;
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

  int createInputImage(const cv::Mat &src);
  int getOutputImage(cv::Mat &dst);

  int getPyramidOutputImage(OutputPyramid *output);

 public:
  int src_w;
  int src_h;
  int dst_w;
  int dst_h;
  int rotate = 0;
  CropRect roi;
  PyramidParam pym_param;

 private:
  shmfifo_t fifo;
  OutputImage *hobotcv_output;       //映射到输出图片共享内存
  OutputPyramid *hobotcv_pymOutput;  //映射到金字塔输出shmem
  sem_t *hobotcv_sem_output;
  std::string str_stamp;
  int output_shm_Index = -1;
  int output_pym_Index = -1;
};

}  // namespace hobot_cv

#endif  // HOBOTCV_FRONT_H
