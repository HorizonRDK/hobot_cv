# 功能介绍

hobotcv_benchmark是hobot_cv vps和bpu以及opencv对图片处理耗时统计的工具。hobotcv_benchmark默认每调用1000次输出一次帧率以及单帧延时的最大值、最小值和平均值。用户可以通过更改启动参数，配置不同的加速方式以及不同的图片操作。图像数据来源于本地图片回灌。

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
| speedup_type    | 图片处理加速方式                |   0:vps 1:bpu 2:opencv        |      0          |
| static_cycle    | 一个周期处理图片个数            |   int                         |      1000         |


## 运行

运行方式1，使用ros2 run启动：

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# 测试bpu加速方式进行resize的benchmark数据，接口类型为cv::Mat数据接口
ros2 run hobot_cv hobotcv_benchmark --ros-args -p speedup_type:=1 -p img_fmt:=0 -p process_type:=0

```

运行方式2，使用launch文件启动：
```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# 启动launch文件
ros2 launch hobot_cv hobot_cv_benchmark.launch.py

# 统计opencv resize的耗时
ros2 launch hobot_cv hobot_cv_benchmark.launch.py speedup_type:=2 process_type:=0 image_file:=config/test.jpg dst_width:=960 dst_height:=540

```

## 结果分析
启动命令：ros2 launch hobot_cv hobot_cv_benchmark.launch.py
输出结果：
```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [hobotcv_benchmark-1]: process started with pid [5796]
[hobotcv_benchmark-1] [WARN] [1666377438.249075414] [benchmark]: This is hobot_cv benchmark!
[hobotcv_benchmark-1] [ERROR]["vps"][vps/hb_vps_api.c:191] [87.462736]HB_VPS_StopGrp[191]: VPS StopGrp err: bad group num 4!
[hobotcv_benchmark-1]
[hobotcv_benchmark-1] [ERROR]["vps"][vps/hb_vps_api.c:87] [87.462805]HB_VPS_DestroyGrp[87]: VPS destroy grp error: unexist group
[hobotcv_benchmark-1]
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 Throughput 88.4777fps
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 latency: [avg: 11.295ms,  max: 11.938ms,  min: 11.12ms]
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 Throughput 89.3716fps
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 latency: [avg: 11.1855ms,  max: 11.387ms,  min: 11.118ms]
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 Throughput 88.586fps
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 latency: [avg: 11.2793ms,  max: 12.069ms,  min: 11.102ms]
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 Throughput 89.4254fps
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 latency: [avg: 11.178ms,  max: 11.418ms,  min: 11.102ms]
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 Throughput 89.3923fps
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 latency: [avg: 11.1729ms,  max: 12.385ms,  min: 11.094ms]
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 Throughput 88.6265fps
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 latency: [avg: 11.2744ms,  max: 12.102ms,  min: 11.082ms]
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 Throughput 89.464fps
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 latency: [avg: 11.1735ms,  max: 11.423ms,  min: 11.102ms]
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 Throughput 88.7525fps
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 latency: [avg: 11.2604ms,  max: 11.837ms,  min: 11.111ms]

```
根据log输出，benchmark测试了以hobotcv的vps加速方式，接口输入图片格式为cv::Mat。统计了将1920x1080的图片resize到960x540，每1000次耗时的平均值，最大值以及最小值，同时也输出了hobot_cv的输出帧率。

## 注意事项

运行时需要注意锁频：
```
sudo bash -c 'echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor'
```

查看benchmark进程cpu占用：
1. 先使用 `ps -ef | grep hobotcv_benchmark` 查看进程id
2. 再通过 `top -p 进程id` 查看进程cpu占用以及内存占用

查看benchmark进程bpu占用：`hrut_somstatus`


## 不同负载测试对比

使用hobot_cv benchmark工具测试，读取本地1920x1080分辨率图片，将1920x1080分辨率resize到512x512，一个周期处理图片个数static_cycle设置为1000。
分别在以下case中统计VPS、BPU和OPENCV耗时最大值、最小值、平均值，输出帧率以及资源占比。统计数据不包含第一次处理需要配置硬件属性的时间。
- case1：在无负载情况下测试
- case2：启动测试程序，使CPU每个核的CPU占比约为50%。在CPU负载情况下测试。
- case3：VPS负载，在已经启动了两个hobot_cv VPS加速程序情况下测试。
- case4：BPU负载35%（启动dnn程序推理fcos模型，CPU负载30%）
- case5：BPU负载50%（启动dnn程序推理yolov5模型，CPU负载16.6%）

<table>
  <tr>
    <th></th>
    <th colspan="3">无负载</th>
    <th colspan="3">CPU负载50%</th>
    <th colspan="3">VPS负载</th>
    <th colspan="3">BPU负载35%</th>
    <th colspan="3">BPU负载50%</th>
  </tr >
  <tr>
    <td >统计类型</td>
    <td>VPS加速</td>
    <td>BPU加速</td>
    <td>opencv</td>
    <td>VPS加速</td>
    <td>BPU加速</td>
    <td>opencv</td>
    <td>VPS加速</td>
    <td>BPU加速</td>
    <td>opencv</td>
    <td>VPS加速</td>
    <td>BPU加速</td>
    <td>opencv</td>
    <td>VPS加速</td>
    <td>BPU加速</td>
    <td>opencv</td>
  </tr >
  <tr >
    <td>最大值(ms)</td>
    <td>11.699</td>
    <td>8.18</td>
    <td>19.326</td>
    <td>18.906</td>
    <td>14.899</td>
    <td>39.086</td>
    <td>26.711</td>
    <td>11.683</td>
    <td>18.38</td>
    <td>13.667</td>
    <td>21.412</td>
    <td>20.293</td>
    <td>10.817</td>
    <td>63.973</td>
    <td>19.748</td>
  </tr>
  <tr >
    <td>最小值(ms)</td>
    <td>10.752</td>
    <td>5.562</td>
    <td>7.397</td>
    <td>10.819</td>
    <td>5.602</td>
    <td>7.616</td>
    <td>11.124</td>
    <td>5.827</td>
    <td>7.381</td>
    <td>11.314</td>
    <td>5.831</td>
    <td>7.52</td>
    <td>13.383</td>
    <td>5.768</td>
    <td>7.55</td>
  </tr>
  <tr >
    <td>平均值(ms)</td>
    <td>10.8882</td>
    <td>5.79068</td>
    <td>8.21311</td>
    <td>10.946</td>
    <td>5.945</td>
    <td>16.55</td>
    <td>15.66</td>
    <td>6.6787</td>
    <td>9.0663</td>
    <td>11.658</td>
    <td>8.418</td>
    <td>10.264</td>
    <td>11.333</td>
    <td>10.714</td>
    <td>9.2706</td>
  </tr>
  <tr >
    <td>帧率(fps)</td>
    <td>91.8155</td>
    <td>172.546</td>
    <td>121.567</td>
    <td>91.2686</td>
    <td>164.10</td>
    <td>60.365</td>
    <td>63.81</td>
    <td>149.57</td>
    <td>110.17</td>
    <td>85.736</td>
    <td>118.59</td>
    <td>97.328</td>
    <td>88.202</td>
    <td>92.884</td>
    <td>107.73</td>
  </tr>
  <tr >
    <td>CPU占用(%)</td>
    <td>20.3</td>
    <td>71.1</td>
    <td>380</td>
    <td>20.3</td>
    <td>67.9</td>
    <td>210</td>
    <td>17.3</td>
    <td>71.8</td>
    <td>355.6</td>
    <td>33.1</td>
    <td>54.5</td>
    <td>324.8</td>
    <td>23.2</td>
    <td>44.2</td>
    <td>350.2</td>
  </tr>
  <tr >
    <td>30fps时CPU占用(%)</td>
    <td>6.63</td>
    <td>12.36</td>
    <td>93.82</td>
    <td>6.67</td>
    <td>12.41</td>
    <td>104.37</td>
    <td>8.13</td>
    <td>14.42</td>
    <td>96.89</td>
    <td>11.57</td>
    <td>13.78</td>
    <td>100.12</td>
    <td>7.89</td>
    <td>14.27</td>
    <td>97.52</td>
  </tr>
  <tr >
    <td>Ratio bpu0</td>
    <td>0</td>
    <td>35</td>
    <td>0</td>
    <td>0</td>
    <td>34</td>
    <td>0</td>
    <td>0</td>
    <td>34</td>
    <td>0</td>
    <td>35</td>
    <td>61</td>
    <td>34</td>
    <td>44</td>
    <td>62</td>
    <td>41</td>
  </tr>
  <tr >
    <td>Ratio bpu1</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>33</td>
    <td>35</td>
    <td>33</td>
    <td>43</td>
    <td>49</td>
    <td>47</td>
  </tr>
</table>
