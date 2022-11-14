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

#include "hobotcv_imgproc/hobotcv_imgproc.h"

#include <iostream>

#include "hobotcv_imgproc/hobotcv_front.h"
#include "include/utils.h"
#include "rclcpp/rclcpp.hpp"

namespace hobot_cv {

int hobotcv_resize(const cv::Mat &src,
                   int src_h,
                   int src_w,
                   cv::Mat &dst,
                   int dst_h,
                   int dst_w,
                   HobotcvSpeedUpType type) {
  if (type == HOBOTCV_AUTO) {
    hobotcv_front hobotcv;
    auto ret = hobotcv.prepareResizeParam(src_w, src_h, dst_w, dst_h, false);
    if (ret == 0) {
      return hobotcv_vps_resize(
          src, dst, dst_h, dst_w, cv::Range(0, 0), cv::Range(0, 0));
    }
  } else if (type == HOBOTCV_VPS) {
    return hobotcv_vps_resize(
        src, dst, dst_h, dst_w, cv::Range(0, 0), cv::Range(0, 0));
  }
  auto ret = prepareBpuResizeParam(src_w, src_h, dst_w, dst_h);
  if (ret != 0) {
    return ret;
  }
  hbDNNTensor input_tensor;
  prepare_nv12_tensor_without_padding(
      reinterpret_cast<const char *>(src.data), src_h, src_w, &input_tensor);
  // Prepare output tensor
  hbDNNTensor output_tensor;
  prepare_nv12_tensor_without_padding(dst_h, dst_w, &output_tensor);
  // resize
  hbDNNResizeCtrlParam ctrl = {
      HB_BPU_CORE_0, 0, HB_DNN_RESIZE_TYPE_BILINEAR, 0, 0, 0, 0};
  hbDNNTaskHandle_t task_handle = nullptr;

  ret =
      hbDNNResize(&task_handle, &output_tensor, &input_tensor, nullptr, &ctrl);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "hbDNNResize failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return ret;
  }
  ret = hbDNNWaitTaskDone(task_handle, 0);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "hbDNNWaitTaskDone failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return ret;
  }
  ret = hbDNNReleaseTask(task_handle);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "release task  failed!!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return ret;
  }

  size_t size = dst_h * dst_w * 3 / 2;
  dst = cv::Mat(dst_h * 3 / 2, dst_w, CV_8UC1);
  memcpy(dst.data, output_tensor.sysMem[0].virAddr, size);
  ret = hbSysFreeMem(&(input_tensor.sysMem[0]));
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "Free input_tensor mem failed!!");
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return ret;
  }
  ret = hbSysFreeMem(&(output_tensor.sysMem[0]));
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"),
                 "Free output_tensor mem failed!!");
    return ret;
  }
  return 0;
}

cv::Mat hobotcv_crop(const cv::Mat &src,
                     int src_h,
                     int src_w,
                     int dst_h,
                     int dst_w,
                     const cv::Range &rowRange,
                     const cv::Range &colRange,
                     HobotcvSpeedUpType type) {
  if (type == HOBOTCV_AUTO) {
    hobotcv_front hobotcv;
    auto ret = hobotcv.prepareCropRoi(
        src_h, src_w, dst_w, dst_h, rowRange, colRange, false);
    if (ret == 0) {
      ret = hobotcv.prepareResizeParam(src_w, src_h, dst_w, dst_h, false);
      if (ret == 0) {
        cv::Mat dst;
        hobotcv_vps_resize(src, dst, dst_h, dst_w, rowRange, colRange);
        return dst;
      }
    }
  }
  if (type == HOBOTCV_VPS) {
    cv::Mat dst;
    hobotcv_vps_resize(src, dst, dst_h, dst_w, rowRange, colRange);
    return dst;
  }
  cv::Mat dst(dst_h * 3 / 2, dst_w, CV_8UC1);
  if (rowRange.end > src_h || colRange.end > src_w || rowRange.start < 0 ||
      colRange.start < 0) {  // crop区域要在原图范围内
    RCLCPP_ERROR(
        rclcpp::get_logger("hobot_cv crop"),
        "Invalid Range data, rowRange.start:%d rowRange.end:%d "
        "colRange.start: %d colRange.end: %d"
        "rowRange should be in [0, %d) and colRange should be in [0, %d)",
        rowRange.start,
        rowRange.end,
        colRange.start,
        colRange.end,
        src_h,
        src_w);
    return dst;
  }

  hbDNNRoi roi;
  int range_h = 0, range_w = 0;
  if (colRange.end - colRange.start <= 0 ||
      rowRange.end - rowRange.start <= 0) {
    roi.left = 0;
    roi.top = 0;
    roi.right = 0;
    roi.bottom = 0;
  } else {
    roi.left = colRange.start;
    roi.top = rowRange.start;
    roi.right = (colRange.end - 1) < 0 ? 0 : (colRange.end - 1);
    roi.bottom = (rowRange.end - 1) < 0 ? 0 : (rowRange.end - 1);
    range_h = rowRange.end - rowRange.start;
    range_w = colRange.end - colRange.start;
  }

  if (range_h == dst_h && range_w == dst_w) {
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
    return dst;
  }

  auto ret = prepareBpuResizeParam(range_w, range_h, dst_w, dst_h);
  if (ret != 0) {
    return dst;
  }

  hbDNNTensor input_tensor;
  prepare_nv12_tensor_without_padding(
      reinterpret_cast<const char *>(src.data), src_h, src_w, &input_tensor);
  // Prepare output tensor
  hbDNNTensor output_tensor;
  prepare_nv12_tensor_without_padding(dst_h, dst_w, &output_tensor);

  // resize
  hbDNNResizeCtrlParam ctrl = {
      HB_BPU_CORE_0, 0, HB_DNN_RESIZE_TYPE_BILINEAR, 0, 0, 0, 0};
  hbDNNTaskHandle_t task_handle = nullptr;

  ret = 0;
  if (range_w == 0 || range_h == 0) {
    ret = hbDNNResize(
        &task_handle, &output_tensor, &input_tensor, nullptr, &ctrl);
  } else {
    ret = hbDNNResize(&task_handle, &output_tensor, &input_tensor, &roi, &ctrl);
  }

  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv crop"), "hbDNNResize failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return dst;
  }
  ret = hbDNNWaitTaskDone(task_handle, 0);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv crop"),
                 "hbDNNWaitTaskDone failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return dst;
  }
  hbDNNReleaseTask(task_handle);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv crop"), "release task failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return dst;
  }

  size_t size = dst_h * dst_w * 3 / 2;
  memcpy(dst.data, output_tensor.sysMem[0].virAddr, size);
  ret = hbSysFreeMem(&(input_tensor.sysMem[0]));
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv crop"),
                 "Free input_tensor mem failed!");
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return dst;
  }
  ret = hbSysFreeMem(&(output_tensor.sysMem[0]));
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv crop"),
                 "Free output_tensor mem failed!");
    return dst;
  }
  return dst;
}

int hobotcv_rotate(const cv::Mat &src, cv::Mat &dst, ROTATION_E rotation) {
  hobotcv_front hobotcv;
  int ret =
      hobotcv.prepareRotateParam(src.cols, src.rows * 2 / 3, (int)rotation);
  if (ret != 0) {
    return -1;
  }
  hobotcv.src_w = src.cols;
  hobotcv.src_h = src.rows * 2 / 3;
  hobotcv.dst_w = src.cols;
  hobotcv.dst_h = src.rows * 2 / 3;

  ret = hobotcv.groupScheduler();
  if (ret != 0) {
    return -1;
  }
  ret = hobotcv.sendVpsFrame(
      reinterpret_cast<const char *>(src.data), hobotcv.src_h, hobotcv.src_w);
  if (ret != 0) {
    return -1;
  }
  ret = hobotcv.getChnFrame(dst);
  if (ret != 0) {
    return -1;
  }
  return 0;
}

int hobotcv_imgproc(const cv::Mat &src,
                    cv::Mat &dst,
                    int dst_h,
                    int dst_w,
                    ROTATION_E rotate,
                    const cv::Range &rowRange,
                    const cv::Range &colRange) {
  int src_h = src.rows * 2 / 3;
  int src_w = src.cols;

  hobotcv_front hobotcv;
  int ret =
      hobotcv.prepareCropRoi(src_h, src_w, dst_w, dst_h, rowRange, colRange);
  if (ret != 0) {
    return -1;
  }

  ret = hobotcv.prepareResizeParam(src_w, src_h, dst_w, dst_h);
  if (0 != ret) {
    return -1;
  }

  ret = hobotcv.prepareRotateParam(dst_w, dst_h, (int)rotate);
  if (ret != 0) {
    return -1;
  }
  if (hobotcv.roi.height == dst_h && hobotcv.roi.width == dst_w &&
      hobotcv.rotate == 0) {  // only crop
    auto srcdata = reinterpret_cast<const uint8_t *>(src.data);
    dst = cv::Mat(dst_h * 3 / 2, dst_w, CV_8UC1);
    auto dstdata = dst.data;
    // copy y
    for (int h = 0; h < dst_h; ++h) {
      auto *raw = dstdata + h * dst_w;
      auto *src = srcdata + (h + hobotcv.roi.y) * src_w + hobotcv.roi.x;
      memcpy(raw, src, dst_w);
    }

    // copy uv
    auto uv_data = srcdata + src_h * src_w;
    auto dstuvdata = dstdata + dst_h * dst_w;
    for (int32_t h = 0; h < dst_h / 2; ++h) {
      auto *raw = dstuvdata + h * dst_w;
      auto *src = uv_data + (h + (hobotcv.roi.y / 2)) * src_w + hobotcv.roi.x;
      memcpy(raw, src, dst_w);
    }
    return 0;
  }

  ret = hobotcv.groupScheduler();
  if (ret != 0) {
    return -1;
  }
  ret = hobotcv.sendVpsFrame(
      reinterpret_cast<const char *>(src.data), src_h, src_w);
  if (ret != 0) {
    return -1;
  }
  ret = hobotcv.getChnFrame(dst);
  if (ret != 0) {
    return -1;
  }
  return 0;
}

int hobotcv_pymscale(const cv::Mat &src,
                     OutputPyramid *output,
                     const PyramidAttr &attr) {
  int src_h = src.rows * 2 / 3;
  int src_w = src.cols;
  hobotcv_front hobotcv;
  auto ret = hobotcv.preparePymraid(src_h, src_w, attr);
  if (0 != ret) {
    return -1;
  }

  ret = hobotcv.groupScheduler();
  if (ret != 0) {
    return -1;
  }
  ret = hobotcv.sendVpsFrame(
      reinterpret_cast<const char *>(src.data), src_h, src_w);
  if (ret != 0) {
    return -1;
  }
  ret = hobotcv.getPyramidOutputImage(output);
  if (ret != 0) {
    return -1;
  }
  return 0;
}

HobotcvImagePtr hobotcv_BorderPadding(const char *src,
                                      const int &src_h,
                                      const int &src_w,
                                      const HobotcvPaddingType type,
                                      const PaddingArea &area,
                                      const uint8_t value) {
  if (!check_padding_area(area.top,
                          area.bottom,
                          area.left,
                          area.right,
                          src_h,
                          src_w,
                          (int)type)) {
    return nullptr;
  }
  if (type == HobotcvPaddingType::HOBOTCV_CONSTANT) {
    return hobotcv_constant_padding(
        src, src_h, src_w, area.top, area.bottom, area.left, area.right, value);
  } else if (type == HobotcvPaddingType::HOBOTCV_REPLICATE) {
    return hobotcv_replicate_padding(
        src, src_h, src_w, area.top, area.bottom, area.left, area.right);
  } else if (type == HobotcvPaddingType::HOBOTCV_REFLECT) {
    return hobotcv_reflect_padding(
        src, src_h, src_w, area.top, area.bottom, area.left, area.right);
  }
  return nullptr;
}

std::shared_ptr<ImageInfo> hobotcv_resize(const char *src,
                                          int src_h,
                                          int src_w,
                                          int dst_h,
                                          int dst_w,
                                          HobotcvSpeedUpType type) {
  bool vps_resize = false;
  int out_h = dst_h, out_w = dst_w;
  if (type == HOBOTCV_AUTO) {
    hobotcv_front hobotcv;
    auto ret = hobotcv.prepareResizeParam(src_w, src_h, dst_w, dst_h, false);
    if (ret == 0) {
      vps_resize = true;
    }
  } else if (type == HOBOTCV_VPS) {
    vps_resize = true;
  }
  if (vps_resize) {  //使用vps进行resize，调用hobotcv_vps_resize
    auto *dst_sysMem = hobotcv_vps_resize(
        src, src_h, src_w, out_h, out_w, cv::Range(0, 0), cv::Range(0, 0));
    if (dst_sysMem == nullptr) {
      return nullptr;
    }
    auto imageInfo = new ImageInfo;
    imageInfo->width = out_w;
    imageInfo->height = out_h;
    imageInfo->imageAddr = dst_sysMem->virAddr;
    return std::shared_ptr<ImageInfo>(imageInfo,
                                      [dst_sysMem](ImageInfo *imageInfo) {
                                        hbSysFreeMem(dst_sysMem);
                                        delete dst_sysMem;
                                        delete imageInfo;
                                      });
  }
  auto ret = prepareBpuResizeParam(src_w, src_h, dst_w, dst_h);
  if (ret != 0) {
    return nullptr;
  }
  hbDNNTensor input_tensor;
  prepare_nv12_tensor_without_padding(src, src_h, src_w, &input_tensor);
  // Prepare output tensor
  hbDNNTensor output_tensor;
  prepare_nv12_tensor_without_padding(dst_h, dst_w, &output_tensor);
  // resize
  hbDNNResizeCtrlParam ctrl = {
      HB_BPU_CORE_0, 0, HB_DNN_RESIZE_TYPE_BILINEAR, 0, 0, 0, 0};
  hbDNNTaskHandle_t task_handle = nullptr;

  ret =
      hbDNNResize(&task_handle, &output_tensor, &input_tensor, nullptr, &ctrl);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "hbDNNResize failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return nullptr;
  }
  ret = hbDNNWaitTaskDone(task_handle, 0);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "hbDNNWaitTaskDone failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return nullptr;
  }
  ret = hbDNNReleaseTask(task_handle);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv"), "release task failed!!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return nullptr;
  }

  size_t size = dst_h * dst_w * 3 / 2;
  auto *dst_sysMem = new hbSysMem;
  hbSysAllocCachedMem(dst_sysMem, size);
  memcpy(dst_sysMem->virAddr, output_tensor.sysMem[0].virAddr, size);
  hbSysFlushMem(dst_sysMem, HB_SYS_MEM_CACHE_CLEAN);
  hbSysFreeMem(&(input_tensor.sysMem[0]));
  hbSysFreeMem(&(output_tensor.sysMem[0]));
  auto imageInfo = new ImageInfo;
  imageInfo->width = dst_w;
  imageInfo->height = dst_h;
  imageInfo->imageAddr = dst_sysMem->virAddr;
  return std::shared_ptr<ImageInfo>(imageInfo,
                                    [dst_sysMem](ImageInfo *imageInfo) {
                                      hbSysFreeMem(dst_sysMem);
                                      delete dst_sysMem;
                                      delete imageInfo;
                                    });
}

std::shared_ptr<ImageInfo> hobotcv_crop(const char *src,
                                        int src_h,
                                        int src_w,
                                        int dst_h,
                                        int dst_w,
                                        const cv::Range &rowRange,
                                        const cv::Range &colRange,
                                        HobotcvSpeedUpType type) {
  bool vps_resize = false;
  int out_h = dst_h, out_w = dst_w;
  if (type == HOBOTCV_AUTO) {
    hobotcv_front hobotcv;
    auto ret = hobotcv.prepareCropRoi(
        src_h, src_w, dst_w, dst_h, rowRange, colRange, false);
    if (ret == 0) {
      ret = hobotcv.prepareResizeParam(src_w, src_h, dst_w, dst_h, false);
      if (ret == 0) {
        vps_resize = true;
      }
    }
  } else if (type == HOBOTCV_VPS) {
    vps_resize = true;
  }
  if (vps_resize) {  //使用vps进行resize，调用hobotcv_vps_resize
    auto *dst_SysMem =
        hobotcv_vps_resize(src, src_h, src_w, out_h, out_w, rowRange, colRange);
    if (dst_SysMem == nullptr) {
      return nullptr;
    }
    auto imageInfo = new ImageInfo;
    imageInfo->width = out_w;
    imageInfo->height = out_h;
    imageInfo->imageAddr = dst_SysMem->virAddr;
    return std::shared_ptr<ImageInfo>(imageInfo,
                                      [dst_SysMem](ImageInfo *imageInfo) {
                                        hbSysFreeMem(dst_SysMem);
                                        delete dst_SysMem;
                                        delete imageInfo;
                                      });
  }

  if (rowRange.end > src_h || colRange.end > src_w || rowRange.start < 0 ||
      colRange.start < 0) {  // crop区域要在原图范围内
    RCLCPP_ERROR(
        rclcpp::get_logger("hobot_cv crop"),
        "Invalid Range data, rowRange.start:%d rowRange.end:%d "
        "colRange.start: %d colRange.end: %d"
        "rowRange should be in [0, %d) and colRange should be in [0, %d)",
        rowRange.start,
        rowRange.end,
        colRange.start,
        colRange.end,
        src_h,
        src_w);
    return nullptr;
  }

  hbDNNRoi roi;
  int range_h = 0, range_w = 0;
  if (colRange.end - colRange.start <= 0 ||
      rowRange.end - rowRange.start <= 0) {
    roi.left = 0;
    roi.top = 0;
    roi.right = 0;
    roi.bottom = 0;
  } else {
    roi.left = colRange.start;
    roi.top = rowRange.start;
    roi.right = (colRange.end - 1) < 0 ? 0 : (colRange.end - 1);
    roi.bottom = (rowRange.end - 1) < 0 ? 0 : (rowRange.end - 1);
    range_h = rowRange.end - rowRange.start;
    range_w = colRange.end - colRange.start;
  }

  if (range_h == dst_h && range_w == dst_w) {
    auto srcdata = reinterpret_cast<const uint8_t *>(src);
    size_t dst_size = dst_h * dst_w * 3 / 2;
    auto *dst_SysMem = new hbSysMem;
    hbSysAllocCachedMem(dst_SysMem, dst_size);
    auto dstdata = reinterpret_cast<uint8_t *>(dst_SysMem->virAddr);
    // copy y
    for (int h = 0; h < dst_h; ++h) {
      auto *raw = dstdata + h * dst_w;
      auto *src_y = srcdata + (h + roi.top) * src_w + roi.left;
      memcpy(raw, src_y, dst_w);
    }

    // copy uv
    auto uv_data = srcdata + src_h * src_w;
    auto dstuvdata = dstdata + dst_h * dst_w;
    for (int32_t h = 0; h < dst_h / 2; ++h) {
      auto *raw = dstuvdata + h * dst_w;
      auto *src_uv = uv_data + (h + (roi.top / 2)) * src_w + roi.left;
      memcpy(raw, src_uv, dst_w);
    }
    hbSysFlushMem(dst_SysMem, HB_SYS_MEM_CACHE_CLEAN);
    auto imageInfo = new ImageInfo;
    imageInfo->width = dst_w;
    imageInfo->height = dst_h;
    imageInfo->imageAddr = dst_SysMem->virAddr;
    return std::shared_ptr<ImageInfo>(imageInfo,
                                      [dst_SysMem](ImageInfo *imageInfo) {
                                        hbSysFreeMem(dst_SysMem);
                                        delete dst_SysMem;
                                        delete imageInfo;
                                      });
  }

  auto ret = prepareBpuResizeParam(range_w, range_h, dst_w, dst_h);
  if (ret != 0) {
    return nullptr;
  }

  hbDNNTensor input_tensor;
  prepare_nv12_tensor_without_padding(src, src_h, src_w, &input_tensor);
  // Prepare output tensor
  hbDNNTensor output_tensor;
  prepare_nv12_tensor_without_padding(dst_h, dst_w, &output_tensor);

  // resize
  hbDNNResizeCtrlParam ctrl = {
      HB_BPU_CORE_0, 0, HB_DNN_RESIZE_TYPE_BILINEAR, 0, 0, 0, 0};
  hbDNNTaskHandle_t task_handle = nullptr;

  ret = 0;
  if (range_w == 0 || range_h == 0) {
    ret = hbDNNResize(
        &task_handle, &output_tensor, &input_tensor, nullptr, &ctrl);
  } else {
    ret = hbDNNResize(&task_handle, &output_tensor, &input_tensor, &roi, &ctrl);
  }

  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv crop"), "hbDNNResize failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return nullptr;
  }
  ret = hbDNNWaitTaskDone(task_handle, 0);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv crop"),
                 "hbDNNWaitTaskDone failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return nullptr;
  }
  hbDNNReleaseTask(task_handle);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_cv crop"), "release task failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
    return nullptr;
  }

  size_t size = dst_h * dst_w * 3 / 2;
  auto *dst_SysMem = new hbSysMem;
  hbSysAllocCachedMem(dst_SysMem, size);
  auto dstdata = dst_SysMem->virAddr;
  memcpy(dstdata, output_tensor.sysMem[0].virAddr, size);
  hbSysFlushMem(dst_SysMem, HB_SYS_MEM_CACHE_CLEAN);

  hbSysFreeMem(&(input_tensor.sysMem[0]));
  hbSysFreeMem(&(output_tensor.sysMem[0]));

  auto imageInfo = new ImageInfo;
  imageInfo->width = dst_w;
  imageInfo->height = dst_h;
  imageInfo->imageAddr = dst_SysMem->virAddr;
  return std::shared_ptr<ImageInfo>(imageInfo,
                                    [dst_SysMem](ImageInfo *imageInfo) {
                                      hbSysFreeMem(dst_SysMem);
                                      delete dst_SysMem;
                                      delete imageInfo;
                                    });
}

std::shared_ptr<ImageInfo> hobotcv_rotate(const char *src,
                                          int src_h,
                                          int src_w,
                                          ROTATION_E rotate) {
  hobotcv_front hobotcv;
  int ret = hobotcv.prepareRotateParam(src_w, src_h, (int)rotate);
  if (ret != 0) {
    return nullptr;
  }
  hobotcv.src_w = src_w;
  hobotcv.src_h = src_h;
  hobotcv.dst_w = src_w;
  hobotcv.dst_h = src_h;

  ret = hobotcv.groupScheduler();
  if (ret != 0) {
    return nullptr;
  }
  ret = hobotcv.sendVpsFrame(src, src_h, src_w);
  if (ret != 0) {
    return nullptr;
  }
  int dst_h = 0, dst_w = 0;
  auto *dst_SysMem = hobotcv.getChnFrame(dst_h, dst_w);
  if (dst_SysMem == nullptr) {
    return nullptr;
  }
  auto imageInfo = new ImageInfo;
  imageInfo->width = dst_w;
  imageInfo->height = dst_h;
  imageInfo->imageAddr = dst_SysMem->virAddr;
  return std::shared_ptr<ImageInfo>(imageInfo,
                                    [dst_SysMem](ImageInfo *imageInfo) {
                                      hbSysFreeMem(dst_SysMem);
                                      delete dst_SysMem;
                                      delete imageInfo;
                                    });
}

std::shared_ptr<ImageInfo> hobotcv_imgproc(const char *src,
                                           int src_h,
                                           int src_w,
                                           int dst_h,
                                           int dst_w,
                                           ROTATION_E rotate,
                                           const cv::Range &rowRange,
                                           const cv::Range &colRange) {
  hobotcv_front hobotcv;
  int ret =
      hobotcv.prepareCropRoi(src_h, src_w, dst_w, dst_h, rowRange, colRange);
  if (ret != 0) {
    return nullptr;
  }

  ret = hobotcv.prepareResizeParam(src_w, src_h, dst_w, dst_h);
  if (0 != ret) {
    return nullptr;
  }

  ret = hobotcv.prepareRotateParam(dst_w, dst_h, (int)rotate);
  if (ret != 0) {
    return nullptr;
  }
  if (hobotcv.roi.height == dst_h && hobotcv.roi.width == dst_w &&
      hobotcv.rotate == 0) {  // only crop
    auto srcdata = reinterpret_cast<const uint8_t *>(src);
    size_t dst_size = dst_h * dst_w * 3 / 2;
    auto *dst_SysMem = new hbSysMem;
    hbSysAllocCachedMem(dst_SysMem, dst_size);
    auto dstdata = reinterpret_cast<uint8_t *>(dst_SysMem->virAddr);
    // copy y
    for (int h = 0; h < dst_h; ++h) {
      auto *raw = dstdata + h * dst_w;
      auto *src_raw = srcdata + (h + hobotcv.roi.y) * src_w + hobotcv.roi.x;
      memcpy(raw, src_raw, dst_w);
    }

    // copy uv
    auto uv_data = srcdata + src_h * src_w;
    auto dstuvdata = dstdata + dst_h * dst_w;
    for (int32_t h = 0; h < dst_h / 2; ++h) {
      auto *raw = dstuvdata + h * dst_w;
      auto *src_raw =
          uv_data + (h + (hobotcv.roi.y / 2)) * src_w + hobotcv.roi.x;
      memcpy(raw, src_raw, dst_w);
    }
    hbSysFlushMem(dst_SysMem, HB_SYS_MEM_CACHE_CLEAN);

    auto imageInfo = new ImageInfo;
    imageInfo->width = dst_w;
    imageInfo->height = dst_h;
    imageInfo->imageAddr = dst_SysMem->virAddr;
    return std::shared_ptr<ImageInfo>(imageInfo,
                                      [dst_SysMem](ImageInfo *imageInfo) {
                                        hbSysFreeMem(dst_SysMem);
                                        delete dst_SysMem;
                                        delete imageInfo;
                                      });
  }

  ret = hobotcv.groupScheduler();
  if (ret != 0) {
    return nullptr;
  }
  ret = hobotcv.sendVpsFrame(src, src_h, src_w);
  if (ret != 0) {
    return nullptr;
  }
  int out_h = 0, out_w = 0;
  auto *dst_SysMem = hobotcv.getChnFrame(out_h, out_w);
  if (dst_SysMem == nullptr) {
    return nullptr;
  }
  auto imageInfo = new ImageInfo;
  imageInfo->width = out_w;
  imageInfo->height = out_h;
  imageInfo->imageAddr = dst_SysMem->virAddr;
  return std::shared_ptr<ImageInfo>(imageInfo,
                                    [dst_SysMem](ImageInfo *imageInfo) {
                                      hbSysFreeMem(dst_SysMem);
                                      delete dst_SysMem;
                                      delete imageInfo;
                                    });
}

int hobotcv_pymscale(const char *src,
                     int src_h,
                     int src_w,
                     OutputPyramid *output,
                     const PyramidAttr &attr) {
  hobotcv_front hobotcv;
  auto ret = hobotcv.preparePymraid(src_h, src_w, attr);
  if (0 != ret) {
    return -1;
  }

  ret = hobotcv.groupScheduler();
  if (ret != 0) {
    return -1;
  }
  ret = hobotcv.sendVpsFrame(src, src_h, src_w);
  if (ret != 0) {
    return -1;
  }
  ret = hobotcv.getPyramidOutputImage(output);
  if (ret != 0) {
    return -1;
  }
  return 0;
}

}  // namespace hobot_cv
