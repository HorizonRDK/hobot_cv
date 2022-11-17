# 功能介绍

hobotcv_benchmark是hobot_cv vps和bpu以及opencv对图片处理耗时统计的工具。每隔33ms调用一次处理接口，每调用100次输出一次耗时最大值，最小值和平均值。

# 编译

## 依赖库

- hobot_cv package

## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0


# 使用介绍

## 依赖

## 参数

| 参数名           | 含义                   | 取值                          | 默认值                |
| --------------  | ---------------------- | ----------------------------- | --------------------- |
| image_file      | 输入图片的路径          | 字符串                         |      config/test.jpg       |
| dst_width       | resize后输出图片宽      | int                          |      960          |
| dst_height      | resize后输出图片高      | int                          |      540          |
| rotation        | 旋转角度                | 90/180/270                  |      180          |
| cv_type         | 图片处理操作         | resize/rotate                  |      resize          |
| interface_type   | hobot_cv接口输入输出图片格式 | 1：cv::Mat  2: nv12       |      2          |
| speed_type       | 图片处理加速方式         |    vps/bpu/opencv               |      vps          |


## 运行

运行方式1，使用ros2 run启动：

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# 测试bpu加速方式进行resize的benchmark数据，接口类型为cv::Mat数据接口
ros2 run hobotcv_benchmark benchmark --ros-args -p speed_type:=bpu -p interface_type:=1

```

运行方式2，使用launch文件启动：
```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# 启动launch文件
ros2 launch hobotcv_benchmark hobot_cv_benchmark.launch.py

# 统计opencv resize的耗时
ros2 launch hobotcv_benchmark hobot_cv_benchmark.launch.py speed_type:=opencv

```

## 注意事项

运行时需要注意锁频：
```
sudo bash -c 'echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor'
```
