Getting Started with hobot_cv
=======

# 功能介绍

hobot_cv package是地平线机器人开发平台的一部分，为应用开发提供类似OpenCV接口。目前实现了图片的crop，resize，crop&resize功能，暂时只支持nv12格式。

# 编译

## 依赖库

- dnn:1.8.4
- easydnn:0.3.3
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
  源码包含**hobot_cv package**，用户可通过hobot_cv提供的接口实现图片的crop，resize。

## 运行
- 编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行。

## X3 Ubuntu系统上运行

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.bash
# config中为example使用的模型，回灌使用的本地图片
# 根据实际安装路径进行拷贝（docker中的安装路径为install/lib/hobot_cv/config/，拷贝命令为cp -r install/lib/hobot_cv/config/ .）。
cp -r install/hobot_cv/lib/hobot_cv/config/ .

# 运行模式1：
使用本地nv12格式图片通过hobot_cv接口实现图片的crop，resize，并以jpg格式存储变换后的图片
ros2 run hobot_cv example
```

## X3 yocto系统上运行

```
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# config中为example使用的模型，回灌使用的本地图片
cp -r install/lib/hobot_cv/config/ .

# 运行模式1：使用本地nv12格式图片通过hobot_cv接口实现图片的crop，resize，并以jpg格式存储变换后的图片
./install/lib/hobot_cv/example

```
