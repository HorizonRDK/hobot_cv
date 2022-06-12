Getting Started with hobot_cv
=======

# 功能介绍

hobot_cv package是地平线机器人开发平台的一部分，为应用开发提供类似OpenCV接口。目前实现了图片的crop，resize，crop&resize功能，暂时只支持nv12格式。

hobot_cv高斯模糊接口，目前只支持bpu计算加速，且输入为320x240的CV_16UC1格式TOF数据，高斯核为3x3，且sigma均为0。

# 编译

## 依赖库

- dnn:1.8.4
- opencv:3.4.5

## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

### X3 Ubuntu系统上编译

1、编译环境确认

- 板端已安装X3 Ubuntu系统。
- 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
- 已安装ROS2编译工具colcon。安装的ROS不包含编译工具colcon，需要手动安装colcon。colcon安装命令：`pip install -U colcon-common-extensions`

2、编译

- 编译命令：`colcon build --packages-select hobot_cv`

### docker交叉编译

1、编译环境确认

- 在docker中编译，并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。

2、编译

- 编译命令：

  ```
  export TARGET_ARCH=aarch64
  export TARGET_TRIPLE=aarch64-linux-gnu
  export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

  colcon build --packages-select hobot_cv \
     --merge-install \
     --cmake-force-configure \
     --cmake-args \
     --no-warn-unused-cli \
     -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
  ```

## 注意事项
  目前hobot_cv接口只支持nv12格式

# 使用介绍

## package说明
  源码包含**hobot_cv package**，用户可通过hobot_cv提供的接口实现图片的crop，resize，高斯滤波。

## 接口说明
### crop&resize

int hobotcv_resize(cv::Mat &src, int src_h, int src_w, cv::Mat &dst, int dst_h, int dst_w);
功能介绍：nv12格式图片的resize功能。
返回值：成功返回0，失败返回非零错误码。
参数：
| 参数名 | 解释                 |
| ------ | -------------------- |
| src    | 原nv12格式的图像矩阵 |
| src_h  | 原图高               |
| sc_w   | 原图宽               |
| dst    | resize后的图像矩阵   |
| dst_h  | resize后的高         |
| dst_w  | resize后的宽         |
         
cv::Mat hobotcv_crop(cv::Mat &src, int src_h, int src_w, int dst_h, int dst_w, const cv::Range& rowRange, const cv::Range& colRange);
功能介绍：将crop区域resize到目标大小。如果crop区域与resize后的大小一致，则只会crop。
返回值：crop&resize之后的nv12图像矩阵。
参数：
| 参数名   | 解释                 |
| -------- | -------------------- |
| src      | 原nv12格式的图像矩阵 |
| src_h    | 原图高               |
| sc_w     | 原图宽               |
| dst_h    | resize后的高         |
| dst_w    | resize后的宽         |
| rowRange | crop的纵向坐标范围   |
| colRange | crop的横向坐标范围   |
注意：crop区域要在图片范围内

### 高斯滤波

int HobotCVGaussianBlurCreate(HobotGaussianBlurParam param, HobotCVGaussianBlurHandle *phandle);
功能介绍：创建高斯滤波的句柄。
返回值：0表示成功，<0表示失败。
参数：

| 参数名   | 解释               |
| -------- | ------------------ |
| param：  | 高斯滤波           |
| --type   | 滤波类型           |
| --width  | 滤波的宽           |
| --height | 滤波的高           |
| --ksizeX | 滤波核的宽         |
| --ksizeY | 滤波核的高         |
| --sigmaX | sigma的宽          |
| --sigmaY | sigma的高          |
| phandle  | 创建成功返回的句柄 |

注：当前版本支持的参数范围如下：

- 滤波类型：高斯滤波
- 支持的数据类型：int16
- 支持的分辨率：320x240。
- 滤波核：高斯3x3
- sigmax: 0.
- sigmay: 0.

int HobotCVGaussianBlurProcess( HobotCVGaussianBlurHandle *phandle，cv::Mat *src，cv::Mat *dst);
功能介绍：创建高斯滤波的句柄。
返回值：0表示成功，<0表示失败。
参数：

| 参数名  | 解释                |
| ------- | ------------------- |
| phandle | 创建成功返回的句柄  |
| src     | 原始的TOF数据矩阵   |
| dst     | 滤波后的TOF数据矩阵 |

int HobotCVGaussianBlurDestroy( HobotCVGaussianBlurHandle *phandle);
功能介绍：创建高斯滤波的句柄。
返回值：0表示成功，<0表示失败。
参数：

| 参数名  | 解释                         |
| ------- | ---------------------------- |
| phandle | 创建成功返回的句柄，用于释放 |

## 运行
- 编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行。

## X3 Ubuntu系统上运行

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.bash
# config中为example使用的模型，回灌使用的本地图片
# 根据实际安装路径进行拷贝（X3 Ubuntu中编译拷贝命令为cp -r install/hobot_cv/lib/hobot_cv/config/ .）。
cp -r install/lib/hobot_cv/config/ .

# 启动crop&resize launch文件
ros2 launch hobot_cv hobot_cv_crop_resize.launch.py

# 启动test_gaussian_blur launch文件
使用本地tof格式图片通过hobot_cv接口实现图片的高斯滤波。
ros2 launch hobot_cv hobot_cv_gaussian_blur.launch.py
```

## X3 yocto系统上运行

```
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# config中为example使用的模型，回灌使用的本地图片
cp -r install/lib/hobot_cv/config/ .

# 使用本地JPEG格式图片通过hobot_cv接口实现图片的crop，resize，并以JPEG格式存储变换后的图片
./install/lib/hobot_cv/example

# 运行模式2：
使用本地tof格式图片通过hobot_cv接口实现图片的高斯滤波。
ros2 run hobot_cv test_gaussian_blur
```

# 结果分析

## X3结果展示

### crop&resize

```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [example-1]: process started with pid [396814]
[example-1] [BPU_PLAT]BPU Platform Version(1.3.1)!
[example-1] [HBRT] set log level as 0. version = 3.13.27
[example-1] [DNN] Runtime version = 1.8.4_(3.13.27 HBRT)
[example-1] 
[example-1] source image config/test.jpg is 1920x1080 pixels
[example-1] resize image to 960x540 pixels, time cost: 30 ms
[example-1] crop image to 960x540 pixels, time cost: 2 ms
[example-1] crop image to 960x540 pixels and resize image to 1920x1080 pixels, time cost: 20 ms
[example-1] 
[INFO] [example-1]: process has finished cleanly [pid 396814]
```

根据log显示，测试程序完成了对本地1920x1080分辨率图片resize，crop，crop&resize的处理。

从1920x1080分辨率图片resize到960x540分辨率，耗时为35 ms。

从1920x1080分辨率图片crop出960x540分辨率的图片，耗时为2 ms。

从1920x1080分辨率图片先crop出960x540分辨率的图片，再将crop出的图片resize到1920x1080分辨率，耗时为22 ms。

原图展示：
![image](./config/test.jpg)

resize效果展示：

![image](./imgs/resize.jpg)

crop效果展示：

![image](./imgs/crop.jpg)

crop&resize效果展示：

![image](./imgs/cropResize.jpg)

### 高斯滤波

```
输出结果：

===================
image name :images/frame1_4.png
infe cost time:1314
guss_time cost time:2685
hobotcv save rate:0.510615

analyse_result start 
---------GaussianBlur
out_filter type:2,cols:320,rows:240,channel:1
cls_filter type:2,cols:320,rows:240,channel:1
out_filter minvalue:96,max:2363
out_filter min,x:319,y:115
out_filter max,x:147,y:239
cls_filter minvalue:96,max:2364
cls_filter min,x:319,y:115
cls_filter max,x:147,y:239

diff diff diff
mat_diff minvalue:0,max:2
mat_diff min,x:2,y:0
mat_diff max,x:110,y:14

error sum:8.46524e+06,max:2,mean_error:0.439232
analyse_result,time_used_ms_end:2
analyse_result end 

------------------------- 
```

其中：

infe cost time:1314　//表示hobotcv加速的高斯滤波耗时1314微秒．

guss_time cost time:2685　//表示opencv的高斯滤波耗时2685微秒．

hobotcv save rate = （guss_time cost time - infe cost time）/ guss_time cost time = 0.510615

从以上比较结果，经过hobotcv加速后性能提升50%。

error sum:8.46524e+06,max:2,mean_error:0.439232　//单张图片总误差是：8.46524e+06，单个像素最大误差是：２，平均误差：0.439232

平均误差　＝　sum / (width * height) = 8.46524e+06 / (320 * 240)
