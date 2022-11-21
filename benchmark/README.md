# 功能介绍

hobotcv_benchmark是hobot_cv vps和bpu以及opencv对图片处理耗时统计的工具。hobotcv_benchmark保持以30fps(摄像头采样输出帧率)速度调用处理接口。每调用100次输出一次耗时最大值，最小值和平均值。用户可以通过更改启动参数，配置不同的加速方式以及不同的图片操作。

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

| 参数名           | 含义                          | 取值                          | 默认值            |
| --------------  | ----------------------------- | ----------------------------- | ----------------- |
| image_file      | 输入图片的路径                 |         字符串                |   config/test.jpg |
| dst_width       | resize后输出图片宽             |          int                  |      960         |
| dst_height      | resize后输出图片高             |          int                  |      540         |
| rotation        | 旋转角度                       |      90/180/270               |      180         |
| process_type    | 图片处理操作                   |   0: resize 1: rotate          |      0           |
| img_fmt         | hobot_cv接口输入输出图片格式    |   0：cv::Mat  1: nv12          |      0           |
| speed_type      | 图片处理加速方式                |   0:vps 1:bpu 2:opencv        |      0            |


## 运行

运行方式1，使用ros2 run启动：

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# 测试bpu加速方式进行resize的benchmark数据，接口类型为cv::Mat数据接口
ros2 run hobot_cv hobotcv_benchmark --ros-args -p speed_type:=1 -p img_fmt:=0 -p process_type:=0

```

运行方式2，使用launch文件启动：
```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# 启动launch文件
ros2 launch hobot_cv hobot_cv_benchmark.launch.py

# 统计opencv resize的耗时
ros2 launch hobot_cv hobot_cv_benchmark.launch.py speed_type:=2 process_type:=0 image_file:=config/test.jpg dst_width:=960 dst_height:=540

```

## 结果分析
启动命令：ros2 launch hobot_cv hobot_cv_benchmark.launch.py
输出结果：
```
[INFO] [launch]: All log files can be found below /root/.ros/log/2022-10-22-02-44-03-046589-ubuntu-22497
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [hobotcv_benchmark-1]: process started with pid [22615]
[hobotcv_benchmark-1] [WARN] [1666377843.894741399] [benchmark]: This is hobot_cv benchmark!
[hobotcv_benchmark-1] hobotcv VPS mat resize 1920x1080 to 960x540 mean cost: 11.1737 min: 11.024 max: 12.006
[hobotcv_benchmark-1] hobotcv VPS mat resize 1920x1080 to 960x540 mean cost: 11.1786 min: 11.008 max: 11.845
[hobotcv_benchmark-1] hobotcv VPS mat resize 1920x1080 to 960x540 mean cost: 11.1808 min: 11.002 max: 12.088
[hobotcv_benchmark-1] hobotcv VPS mat resize 1920x1080 to 960x540 mean cost: 11.1431 min: 10.996 max: 11.949
[hobotcv_benchmark-1] hobotcv VPS mat resize 1920x1080 to 960x540 mean cost: 11.1853 min: 11.011 max: 12.153
[hobotcv_benchmark-1] hobotcv VPS mat resize 1920x1080 to 960x540 mean cost: 11.1249 min: 11.02 max: 11.688
[hobotcv_benchmark-1] hobotcv VPS mat resize 1920x1080 to 960x540 mean cost: 11.1782 min: 10.991 max: 11.835
```
根据log输出，benchmark测试了以hobotcv的vps加速方式，接口输入图片格式为cv::Mat。统计了将1920x1080的图片resize到960x540，每100次耗时的平均值，最大值以及最小值。

## 注意事项

运行时需要注意锁频：
```
sudo bash -c 'echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor'
```

查看benchmark进程cpu占用：
1. 先使用 `ps -ef | grep hobotcv_benchmark` 查看进程id
2. 再通过 `top -p 进程id` 查看进程cpu占用以及内存占用

查看benchmark进程bpu占用：`hrut_somstatus`
