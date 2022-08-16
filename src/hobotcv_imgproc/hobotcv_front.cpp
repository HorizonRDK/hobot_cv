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

#include "utils.h"

namespace hobot_cv {

hobotcv_front::hobotcv_front() {
  group_id = -1;
  roi.cropEnable = 0;
  roi.x = 0;
  roi.y = 0;
  roi.width = 0;
  roi.height = 0;
  pym_param.pymEnable = 0;
  observe = hobotcv_single::getSingleObj();
}

hobotcv_front::~hobotcv_front() {}

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
  if (attr.ds_info[0].factor == 0 || attr.ds_info[4].factor == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "base0 and base 4 must enable!");
    return -1;
  }
  memcpy(&pym_param.attr, &attr, sizeof(PyramidAttr));
  pym_param.pymEnable = 1;
  this->src_h = src_h;
  this->src_w = src_w;
  for (size_t i = 0; i < 24; i++) {
    if (pym_param.attr.ds_info[i].factor != 0) {
      ds_layer_en = i;
    }
  }
  return 0;
}

int hobotcv_front::groupScheduler() {
  processId = getpid();
  //选取crop区域的宽高作为输入源宽高
  src_h = roi.cropEnable == 1 ? roi.height : src_h;
  src_w = roi.cropEnable == 1 ? roi.width : src_w;
  sem_wait(observe->fifo.sem_groups);
  bool have_same_process = false;
  for (int i = 0; i < HOBOTCV_GROUP_SIZE; i++) {
    Group_info_t *group = (Group_info_t *)(observe->fifo.groups) + i;
    if (group->group_state == 1) {
      continue;
    }
    if (group->process_id == processId) {
      if (group->max_h == src_h && src_w == group->max_w) {
        have_same_process = true;
        group_id = i + HOBOTCV_GROUP_BEGIN;
        group->active_time = currentMicroseconds();
      } else {  // group 输入源不同，重新创建group
        group->active_time = 1;
      }
      break;
    }
  }
  if (!have_same_process) {
    auto ret = createGroup();
    if (ret != 0) {
      sem_post(observe->fifo.sem_groups);
      return -1;
    }
  }
  group_sem_wait();

  setVpsChannelAttr();  //设置vps通道属性

  int ret = HB_VPS_EnableChn(group_id, channel_id);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "%d EnableChn: %d failed ret: %d!!",
                 group_id,
                 channel_id,
                 ret);
    group_sem_post();
    sem_post(observe->fifo.sem_groups);
    return -1;
  }
  sem_post(observe->fifo.sem_groups);
  return 0;
}

int hobotcv_front::setVpsChannelAttr() {
  int enScale = 1;
  if (pym_param.pymEnable == 1) {
    setChannelPyramidAttr();
  } else {
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
    int group_index = group_id - HOBOTCV_GROUP_BEGIN;
    Group_info_t *group = (Group_info_t *)(observe->fifo.groups) + group_index;
    if (group->channels[channel_id].output_w != dst_w ||
        group->channels[channel_id].output_h != dst_h) {
      setChannelAttr(enScale);
      group->channels[channel_id].output_w = dst_w;
      group->channels[channel_id].output_h = dst_h;
    }

    group->channels[channel_id].rotation = rotate;
    setChannelRotate();
  }
  return 0;
}

int hobotcv_front::createGroup() {
  auto now_time = currentMicroseconds();
  for (int i = 0; i < HOBOTCV_GROUP_SIZE; i++) {
    Group_info_t *group = (Group_info_t *)(observe->fifo.groups) + i;
    int groupId = i + HOBOTCV_GROUP_BEGIN;
    if (group->active_time == 0) {
      group->group_id = groupId;
      group->max_h = src_h;
      group->max_w = src_w;
      group->active_time = currentMicroseconds();
      group->process_id = processId;
      group->pym_channel = -1;
      group->group_state = 0;
      group_id = groupId;
      break;
    } else if (group->group_state == 1) {  //被系统回收
      group->group_id = groupId;
      group->max_h = src_h;
      group->max_w = src_w;
      group->active_time = currentMicroseconds();
      group->process_id = processId;
      group->group_state = 0;
      group->pym_channel = -1;
      group_id = groupId;
      break;
    } else if (now_time - group->active_time > HOBOTCV_GROUP_OVER_TIME) {
      //超时，hobotcv回收
      HB_VPS_StopGrp(group->group_id);
      HB_VPS_DestroyGrp(group->group_id);
      group->group_id = groupId;
      group->max_h = src_h;
      group->max_w = src_w;
      group->active_time = currentMicroseconds();
      group->process_id = processId;
      group->group_state = 0;
      group->pym_channel = -1;
      memset(group->channels, 0, sizeof(Channel_info_t) * 7);
      group_id = groupId;
      break;
    }
  }

  if (-1 == group_id) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "hobot_cv group is full !!");
    return -1;
  }

  VPS_GRP_ATTR_S grp_attr;
  grp_attr.maxW = src_w;
  grp_attr.maxH = src_h;
  grp_attr.frameDepth = 1;
  auto ret = HB_VPS_CreateGrp(group_id, &grp_attr);
  if (0 != ret) {
    RCLCPP_ERROR(
        rclcpp::get_logger("hobot_cv"), "create group: %d failed!!", group_id);
    return -1;
  }
  observe->group_id = group_id;
  //启用channel 0，1，2，5,pym=3
  groupChn0Init(group_id, grp_attr.maxW, grp_attr.maxH);
  groupChn1Init(group_id, grp_attr.maxW, grp_attr.maxH);
  groupChn2Init(group_id, grp_attr.maxW, grp_attr.maxH);
  groupChn5Init(group_id, grp_attr.maxW, grp_attr.maxH);
  groupPymChnInit(group_id, grp_attr.maxW, grp_attr.maxH);
  ret = HB_VPS_StartGrp(group_id);
  if (0 != ret) {
    RCLCPP_ERROR(
        rclcpp::get_logger("hobot_cv"), "StartGrp: %d failed!!", group_id);
    return -1;
  }
  return 0;
}

int hobotcv_front::setChannelAttr(int enscale) {
  VPS_CHN_ATTR_S chn_attr;
  memset(&chn_attr, 0, sizeof(chn_attr));
  chn_attr.width = dst_w;
  chn_attr.height = dst_h;
  chn_attr.enScale = enscale;
  chn_attr.frameDepth = 1;
  int ret = HB_VPS_SetChnAttr(group_id, channel_id, &chn_attr);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "SetChnAttr failed!!");
    return ret;
  }
  return 0;
}

int hobotcv_front::setChannelRotate() {
  HB_ROTATION_E hb_rotate;
  if (rotate == 0) {
    hb_rotate = HB_ROTATION_E::ROTATION_0;
  } else if (rotate == 90) {
    hb_rotate = HB_ROTATION_E::ROTATION_90;
  } else if (rotate == 180) {
    hb_rotate = HB_ROTATION_E::ROTATION_180;
  } else if (rotate == 270) {
    hb_rotate = HB_ROTATION_E::ROTATION_270;
  }
  int ret = HB_VPS_SetChnRotate(group_id, channel_id, hb_rotate);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "SetChnRotate failed!!");
    return ret;
  }
  return 0;
}

int hobotcv_front::setChannelPyramidAttr() {
  if (pym_param.pymEnable == 1) {
    channel_id = 3;
    VPS_PYM_CHN_ATTR_S pym_chn_attr;
    memset(&pym_chn_attr, 0, sizeof(pym_chn_attr));
    pym_chn_attr.timeout = pym_param.attr.timeout;
    pym_chn_attr.ds_layer_en = ds_layer_en;
    pym_chn_attr.frame_id = 0;
    pym_chn_attr.frameDepth = 1;
    memcpy(pym_chn_attr.ds_info,
           pym_param.attr.ds_info,
           sizeof(pym_chn_attr.ds_info));

    auto ret = HB_VPS_SetPymChnAttr(group_id, channel_id, &pym_chn_attr);
    if (ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "set pym chn failed!!");
      return -1;
    }
  }
  return 0;
}

int hobotcv_front::sendVpsFrame(const cv::Mat &src) {
  int input_w = src.cols;
  int input_h = src.rows * 2 / 3;
  //申请系统内存
  int alloclen = src_h * src_w;
  int ret = HB_SYS_Alloc(&(mmz_paddr[0]), (void **)&(mmz_vaddr[0]), alloclen);
  if (ret == -268500036) {  //需要重新初始化
    RCLCPP_WARN(rclcpp::get_logger("hobot_cv"),
                "vp sys memory is being reinitialized");
    VP_CONFIG_S struVpConf;
    memset(&struVpConf, 0x00, sizeof(VP_CONFIG_S));
    struVpConf.u32MaxPoolCnt = 32;
    HB_VP_SetConfig(&struVpConf);
    ret = HB_VP_Init();
    if (0 != ret) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "HB_VP_Init failed!!");
      return -1;
    }
    ret = HB_SYS_Alloc(&(mmz_paddr[0]), (void **)&(mmz_vaddr[0]), alloclen);
    if (ret != 0) {
      RCLCPP_ERROR(
          rclcpp::get_logger("hobot_cv"), "HB_SYS_Alloc failed ret: %d!!", ret);
      return -1;
    }
  } else if (0 != ret) {
    RCLCPP_ERROR(
        rclcpp::get_logger("hobot_cv"), "HB_SYS_Alloc failed ret: %d!!", ret);
    return -1;
  }
  alloclen = src_h * src_w / 2;
  ret = HB_SYS_Alloc(&(mmz_paddr[1]), (void **)&(mmz_vaddr[1]), alloclen);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "HB_SYS_Alloc failed!!");
    return ret;
  }
  if (roi.cropEnable == 1) {  // crop区域作为vps输入源
    auto srcdata = reinterpret_cast<const uint8_t *>(src.data);
    // copy y
    for (int h = 0; h < roi.height; ++h) {
      auto *raw = mmz_vaddr[0] + h * roi.width;
      auto *src = srcdata + (h + roi.y) * input_w + roi.x;
      memcpy(raw, src, roi.width);
    }
    // copy uv
    auto uv_data = srcdata + input_h * input_w;
    for (int32_t h = 0; h < roi.height / 2; ++h) {
      auto *raw = mmz_vaddr[1] + h * roi.width;
      auto *src = uv_data + (h + (roi.y / 2)) * input_w + roi.x;
      memcpy(raw, src, roi.width);
    }
  } else {
    auto ydata = reinterpret_cast<const uint8_t *>(src.data);
    auto uvdata = ydata + src_h * src_w;
    memcpy(mmz_vaddr[0], ydata, src_h * src_w);
    memcpy(mmz_vaddr[1], uvdata, src_h * src_w / 2);
  }

  hb_vio_buffer_t feedback_buf;
  feedback_buf.img_addr.width = src_w;
  feedback_buf.img_addr.height = src_h;
  feedback_buf.img_addr.stride_size = src_w;

  feedback_buf.img_addr.addr[0] = mmz_vaddr[0];
  feedback_buf.img_addr.addr[1] = mmz_vaddr[1];
  feedback_buf.img_addr.paddr[0] = mmz_paddr[0];
  feedback_buf.img_addr.paddr[1] = mmz_paddr[1];

  ret = HB_VPS_SendFrame(group_id, &feedback_buf, 1000);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "SendFrame failed!!");
    HB_SYS_Free(mmz_paddr[0], mmz_vaddr[0]);
    HB_SYS_Free(mmz_paddr[1], mmz_vaddr[1]);
    return ret;
  }
  HB_SYS_Free(mmz_paddr[0], mmz_vaddr[0]);
  HB_SYS_Free(mmz_paddr[1], mmz_vaddr[1]);
  return 0;
}

int hobotcv_front::getChnFrame(cv::Mat &dst) {
  hb_vio_buffer_t out_buf;
  int ret = HB_VPS_GetChnFrame(group_id, channel_id, &out_buf, 2000);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "get group: %d chn: %d frame failed!!",
                 group_id,
                 channel_id);
    group_sem_post();
    return -1;
  }

  int stride = out_buf.img_addr.stride_size;
  int width = out_buf.img_addr.width;
  int height = out_buf.img_addr.height;

  dst = cv::Mat(height * 3 / 2, width, CV_8UC1);
  copyOutputImage(stride,
                  width,
                  height,
                  out_buf.img_addr,
                  reinterpret_cast<char *>(dst.data));
  HB_VPS_ReleaseChnFrame(group_id, channel_id, &out_buf);
  group_sem_post();
  return 0;
}

int hobotcv_front::getPyramidOutputImage(OutputPyramid *pymOut) {
  pym_buffer_t pym_out;
  auto ret = HB_VPS_GetChnFrame(
      group_id, channel_id, (void *)&pym_out, pym_param.attr.timeout);
  if (ret == 0) {
    pymOut->isSuccess = true;
    int ds_base_index = -1;
    for (int i = 0; i < 24; i++) {
      if (i % 4 == 0) {  // ds 基础层
        ds_base_index++;
        int out_w = pym_out.pym[ds_base_index].width;
        int out_h = pym_out.pym[ds_base_index].height;
        int stride = pym_out.pym[ds_base_index].stride_size;
        if (out_w != 0 && out_h != 0 && pym_param.attr.ds_info[i].factor != 0) {
          pymOut->pym_out[i].width = out_w;
          pymOut->pym_out[i].height = out_h;
          int size = out_w * out_h * 3 / 2;
          pymOut->pym_out[i].img.resize(size);
          copyOutputImage(stride,
                          out_w,
                          out_h,
                          pym_out.pym[ds_base_index],
                          &(pymOut->pym_out[i].img[0]));
        } else {
          pymOut->pym_out[i].width = 0;
          pymOut->pym_out[i].height = 0;
        }
      } else {  // ds roi层
        int roi_index = i - ds_base_index * 4 - 1;
        if (pym_param.attr.ds_info[i].factor != 0) {
          int out_w = pym_out.pym_roi[ds_base_index][roi_index].width;
          int out_h = pym_out.pym_roi[ds_base_index][roi_index].height;
          int stride = pym_out.pym_roi[ds_base_index][roi_index].stride_size;
          pymOut->pym_out[i].width = out_w;
          pymOut->pym_out[i].height = out_h;
          int size = out_w * out_h * 3 / 2;
          pymOut->pym_out[i].img.resize(size);
          copyOutputImage(stride,
                          out_w,
                          out_h,
                          pym_out.pym_roi[ds_base_index][roi_index],
                          &(pymOut->pym_out[i].img[0]));
        } else {
          pymOut->pym_out[i].width = 0;
          pymOut->pym_out[i].height = 0;
        }
      }
    }
    HB_VPS_ReleaseChnFrame(group_id, channel_id, &pym_out);
  } else {
    pymOut->isSuccess = false;
  }

  group_sem_post();
  return 0;
}

int hobotcv_front::group_sem_wait() {
  if (4 == group_id) {
    sem_wait(observe->fifo.sem_group4);
  } else if (5 == group_id) {
    sem_wait(observe->fifo.sem_group5);
  } else if (6 == group_id) {
    sem_wait(observe->fifo.sem_group6);
  } else if (7 == group_id) {
    sem_wait(observe->fifo.sem_group7);
  } else {
    return -1;
  }
  return 0;
}

int hobotcv_front::group_sem_post() {
  if (4 == group_id) {
    sem_post(observe->fifo.sem_group4);
  } else if (5 == group_id) {
    sem_post(observe->fifo.sem_group5);
  } else if (6 == group_id) {
    sem_post(observe->fifo.sem_group6);
  } else if (7 == group_id) {
    sem_post(observe->fifo.sem_group7);
  } else {
    return -1;
  }
  return 0;
}

int hobotcv_front::groupChn0Init(int group_id, int max_w, int max_h) {
  VPS_CHN_ATTR_S chn_attr_max;
  memset(&chn_attr_max, 0, sizeof(chn_attr_max));
  chn_attr_max.width = max_w > 1280 ? 1280 : max_w;
  chn_attr_max.height = max_h > 720 ? 720 : max_h;
  chn_attr_max.enScale = 1;
  chn_attr_max.frameDepth = 1;
  auto ret = HB_VPS_SetChnAttr(group_id, 0, &chn_attr_max);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "group: %d Chn0Init failed! chn_width: %d chn_height: %d",
                 group_id,
                 chn_attr_max.width,
                 chn_attr_max.height);
  }
  return 0;
}

int hobotcv_front::groupChn1Init(int group_id, int max_w, int max_h) {
  VPS_CHN_ATTR_S chn_attr_max;
  memset(&chn_attr_max, 0, sizeof(chn_attr_max));
  chn_attr_max.width = max_w > 2048 ? 2048 : max_w;
  chn_attr_max.height = max_h > 1080 ? 1080 : max_h;
  chn_attr_max.enScale = 1;
  chn_attr_max.frameDepth = 1;
  auto ret = HB_VPS_SetChnAttr(group_id, 1, &chn_attr_max);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "group: %d Chn1Init failed! chn_width: %d chn_height: %d",
                 group_id,
                 chn_attr_max.width,
                 chn_attr_max.height);
  }
  return 0;
}

int hobotcv_front::groupChn2Init(int group_id, int max_w, int max_h) {
  VPS_CHN_ATTR_S chn_attr_max;
  memset(&chn_attr_max, 0, sizeof(chn_attr_max));
  chn_attr_max.width = max_w > 4096 ? 4096 : max_w;
  chn_attr_max.height = max_h > 2156 ? 2156 : max_h;
  chn_attr_max.enScale = 1;
  chn_attr_max.frameDepth = 1;
  auto ret = HB_VPS_SetChnAttr(group_id, 2, &chn_attr_max);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "group: %d Chn2Init failed! chn_width: %d chn_height: %d",
                 group_id,
                 chn_attr_max.width,
                 chn_attr_max.height);
  }
  return 0;
}

int hobotcv_front::groupChn5Init(int group_id, int max_w, int max_h) {
  VPS_CHN_ATTR_S chn_attr_max;
  memset(&chn_attr_max, 0, sizeof(chn_attr_max));
  int max_us_w = max_w * 1.5;
  int max_us_h = max_h * 1.5;
  max_us_w = max_us_w - (max_us_w % 16);
  chn_attr_max.width = max_us_w > 4096 ? 4096 : max_us_w;
  chn_attr_max.height = max_us_h > 2160 ? 2160 : max_us_h;
  chn_attr_max.enScale = 1;
  chn_attr_max.frameDepth = 1;
  auto ret = HB_VPS_SetChnAttr(group_id, 5, &chn_attr_max);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "group: %d Chn5Init failed! chn_width: %d chn_height: %d",
                 group_id,
                 chn_attr_max.width,
                 chn_attr_max.height);
  }
  return 0;
}

int hobotcv_front::groupPymChnInit(int group_id, int max_w, int max_h) {
  VPS_CHN_ATTR_S chn_attr_max;
  memset(&chn_attr_max, 0, sizeof(chn_attr_max));
  chn_attr_max.width = max_w > 2048 ? 2048 : max_w;
  chn_attr_max.height = max_h > 1080 ? 1080 : max_h;
  chn_attr_max.enScale = 1;
  chn_attr_max.frameDepth = 1;
  auto ret = HB_VPS_SetChnAttr(group_id, 3, &chn_attr_max);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "group: %d Chn3Init failed! chn_width: %d chn_height: %d",
                 group_id,
                 chn_attr_max.width,
                 chn_attr_max.height);
    return -1;
  }

  VPS_PYM_CHN_ATTR_S pym_chn_attr;
  memset(&pym_chn_attr, 0, sizeof(pym_chn_attr));
  pym_chn_attr.timeout = 2000;
  pym_chn_attr.ds_layer_en = 23;
  pym_chn_attr.frame_id = 0;
  pym_chn_attr.frameDepth = 1;
  int ds_base_index = -1;
  int roi_maxW = max_w;
  int roi_maxH = max_h;
  for (size_t i = 0; i < 24; i++) {  //初始化roi层
    if (i % 4 == 0) {
      ds_base_index++;
      if (ds_base_index == 0) {
        roi_maxW = max_w;
        roi_maxH = max_h;
      } else {
        roi_maxW = roi_maxW / 2;
        roi_maxH = roi_maxH / 2;
        roi_maxW = (roi_maxW % 2) == 0 ? roi_maxW : (roi_maxW - 1);
        roi_maxH = (roi_maxH % 2) == 0 ? roi_maxH : (roi_maxH - 1);
      }
    }
    if (i % 4 != 0) {
      int roi_tag_x = (roi_maxW - 1) * 64 / (64 + 1) + 1;
      int roi_tag_y = (((roi_maxH / 2 - 1) * 64 / (64 + 1)) + 1) * 2;
      if (roi_tag_x < 48 || roi_tag_y < 32) {
        pym_chn_attr.ds_info[i].factor = 0;
      } else {
        pym_chn_attr.ds_info[i].factor = 1;
      }

      pym_chn_attr.ds_info[i].roi_width = roi_maxW;
      pym_chn_attr.ds_info[i].roi_height = roi_maxH;
    }
  }

  ret = HB_VPS_SetPymChnAttr(group_id, 3, &pym_chn_attr);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "group: %d pymChnInit failed!",
                 group_id);
    return -1;
  }
  return 0;
}

int hobotcv_front::copyOutputImage(
    int stride, int width, int height, address_info_t &img_addr, void *output) {
  if (stride == width) {
    memcpy(output, img_addr.addr[0], width * height);
    memcpy(
        (char *)output + width * height, img_addr.addr[1], width * height / 2);
  } else {
    int i = 0;
    // jump over stride - width Y
    for (i = 0; i < height; i++) {
      memcpy((char *)output + i * width, img_addr.addr[0] + i * stride, width);
    }
    // jump over stride - width UV
    for (i = 0; i < height / 2; i++) {
      memcpy((char *)output + width * height + i * width,
             img_addr.addr[1] + i * stride,
             width);
    }
  }
  return 0;
}

}  // namespace hobot_cv
