English| [简体中文](./README_cn.md)

Getting Started with hobot_cv
=======

# Feature Introduction

The hobot_cv package is part of the Horizon Robotics robot development platform, providing image processing acceleration interfaces for bpu and vps for application development. Currently, functions such as image crop, resize, rotate, border padding, and pyramid scaling have been implemented, supporting only the nv12 format.

hobot_cv Gaussian filtering and mean filtering interfaces support bpu and neon acceleration.

# Development Environment

- Programming Language: C/C++
- Development Platform: X3/X86
- System Version: Ubuntu 20.04
- Compilation Toolchain: Linux GCC 9.3.0/Linaro GCC 9.3.0

# Compilation

- X3 Version: Supports two methods of compiling on the X3 Ubuntu system and cross-compiling using docker on a PC.
- X86 Version: Supports one method of compiling on the X86 Ubuntu system.

## Dependency Libraries

- dnn: 1.8.4
- opencv: 3.4.5

## Compilation on X3 Ubuntu System for X3 Version

1. Confirm Compilation Environment

- X3 Ubuntu system is installed on the board side.
- The current compilation terminal has set the TogetherROS environment variable: `source PATH/setup.bash`. Here, PATH is the installation path of TogetherROS.
- ROS2 compilation tool colcon is installed. If the installed ROS does not include the compilation tool colcon, it needs to be manually installed. The command for installing colcon is: `pip install -U colcon-common-extensions`

2. Compilation

- Compilation command: `colcon build --packages-select hobot_cv`

## Cross-Compilation on X3 Version using Docker

1. Confirm Compilation Environment

- Compile in docker, and TogetherROS is already installed in docker. For docker installation, cross-compilation instructions, TogetherROS compilation, and deployment instructions, see the README.md in the robot development platform robot_dev_config repo.

2. Compilation

- Compilation command:

  ```shell
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

## Compiling X86 Version on X86 Ubuntu System

1. Environment Confirmation

   - X86 Ubuntu Version: Ubuntu 20.04
  
2. Compilation

- Compilation Command:

  ```shell
  colcon build --packages-select hobot_cv \
     --merge-install \
     --cmake-force-configure \
     --cmake-args \
     --no-warn-unused-cli \
     -DPLATFORM_X86=ON \
     -DTHIRD_PARTY=`pwd`/../sysroot_docker
  ```

## Notes
  Currently, hobot_cv crop&resize&rotate only support nv12 format.

  For VPS acceleration, the first processing of different input and output attributes will entail configuring hardware properties, which may take some time. If the properties remain unchanged and hardware direct processing is applied, the processing time will be lower. If there is no corresponding input image for a certain group within 10 seconds after configuring the group properties, hobot_cv will deem that input group inactive, and the group resources will be reused. Group resources are bound to the process that created the group and will only be used within that process. Once the process ends, the group will be automatically released. By default, hobot_cv uses group4, group5, group6, and group7, thus supporting a maximum of four different input properties simultaneously utilizing VPS acceleration.

  VPS acceleration supports input and output images with a maximum resolution of 4096*2160 and a minimum resolution of 32*32. It allows up to 1.5x enlargement and up to 1/8 reduction. The width must be a multiple of 16, and the height must be even.

  For BPU acceleration, the scaling range is [1/185, 256) for dst/src ratio, the input width should be within [16, 4080], and the width must be a multiple of 2. The output size should satisfy w<=4080 and h<=4080.
  When cropping, the crop area must be within the original image.

## User Guide

### Package Description
  The source code includes the **hobot_cv package**, allowing users to implement image crop, resize, rotate, and Gaussian blur through the interfaces provided by hobot_cv.

  hobotcv offers both BPU and VPS acceleration for image processing. Crop and resize can be accelerated using BPU or VPS, while rotate and pyramid are exclusively accelerated using VPS.

  Users can opt for BPU acceleration for less frequent image processing, as it does not require separate hardware property configuration. VPS acceleration, on the other hand, involves longer configuration of hardware properties.
  
  If the camera captures images for crop & resize processing for model inference, you can choose to use VPS acceleration. In this case, the input-output configuration is relatively stable without major changes, and the normal image capture frequency of the camera will not trigger a timeout judgment.

## Interface Description

### resize

int hobotcv_resize(const cv::Mat &src, int src_h, int src_w, cv::Mat &dst, int dst_h, int dst_w, HobotcvSpeedUpType type = HOBOTCV_AUTO);

Function: Resize function for nv12 format images.

Return Value: Returns 0 on success, non-zero on failure.

Parameters:

| Parameter | Explanation               |
| --------- | -------------------------  |
| src       | Original image matrix in nv12 format |
| src_h     | Original image height      |
| sc_w      | Original image width       |
| dst       | Resized image matrix       |
| dst_h     | Resized image height       |
| dst_w     | Resized image width        |
| type      | Enum type for interface acceleration. Default is HOBOTCV_AUTO, which does not support VPS acceleration for input-output. HOBOTCV_VPS for VPS acceleration, HOBOTCV_BPU for BPU acceleration |

std::shared_ptr<ImageInfo> hobotcv_resize(const char *src, int src_h, int src_w, int dst_h, int dst_w, HobotcvSpeedUpType type = HOBOTCV_AUTO);

Function: Resize function for nv12 format images. Input and output image data are addresses of nv12 data.

Return Value: Returns the address of the resized image data on success, nullptr on failure.

Parameters:

| Parameter | Explanation                 |
| --------- | ---------------------------- |
| src       | Address of the input image data |
| src_h     | Input image height           |
| sc_w      | Input image width            |
| dst_h     | Resized image height         |
| dst_w     | Resized image width          |
| type      | Enum type for interface acceleration. Default is HOBOTCV_AUTO, which does not support VPS acceleration for input-output. HOBOTCV_VPS for VPS acceleration, HOBOTCV_BPU for BPU acceleration |

### crop & resize

cv::Mat hobotcv_crop(cv::Mat &src, int src_h, int src_w, int dst_h, int dst_w, const cv::Range& rowRange, const cv::Range& colRange, HobotcvSpeedUpType type = HOBOTCV_AUTO);

Function: Resize the crop region to the target size. If the crop region is the same as the size after resizing, only cropping will occur.

Return Value: The nv12 image matrix after crop & resize.

Note: The crop region should be within the image range.Parameters:
| Parameter Name | Explanation                |
| -------------- | -------------------------- |
| src            | Original image matrix in nv12 format  |
| src_h          | Original image height       |
| src_w          | Original image width        |
| dst_h          | Height after resizing       |
| dst_w          | Width after resizing        |
| rowRange       | Vertical coordinate range for crop    |
| colRange       | Horizontal coordinate range for crop  |
| type           | Enumeration of interface acceleration types, default HOBOTCV_AUTO uses BPU acceleration for input/output that does not meet VPS acceleration. HOBOTCV_VPS for VPS acceleration, HOBOTCV_BPU for BPU acceleration |

std::shared_ptr<ImageInfo> hobotcv_crop(const char *src, int src_h, int src_w, int dst_h, int dst_w, const cv::Range &rowRange, const cv::Range &colRange, HobotcvSpeedUpType type = HOBOTCV_AUTO);

Function: Resize the crop area to the target size. If the crop area is the same as the resized size, it will only crop. Input and output image data is the address of nv12 data.

Return Value: Returns the address of the cropped and resized image data if successful, or nullptr if failed.

Note: The crop area must be within the image range

Parameters:
| Parameter Name | Explanation                |
| -------------- | -------------------------- |
| src            | Input image data address    |
| src_h          | Input image height          |
| src_w          | Input image width           |
| dst_h          | Height after resizing       |
| dst_w          | Width after resizing        |
| rowRange       | Vertical coordinate range for crop    |
| colRange       | Horizontal coordinate range for crop  |
| type           | Enumeration of interface acceleration types, default HOBOTCV_AUTO uses BPU acceleration for input/output that does not meet VPS acceleration. HOBOTCV_VPS for VPS acceleration, HOBOTCV_BPU for BPU acceleration |

### rotate

int hobotcv_rotate(const cv::Mat &src, cv::Mat &dst, ROTATION_E rotate);

Function: Rotate the input image, supporting only 90, 180, 270 degrees rotation. Uses VPS acceleration.

Return Value: Returns 0 if successful, non-zero if failed.

Parameters:
| Parameter Name | Explanation                |
| -------------- | -------------------------- |
| src            | Original image matrix in nv12 format  |
| dst            | Image matrix after rotation   |
| rotate         | Enumeration of rotation angle  |

std::shared_ptr<ImageInfo> hobotcv_rotate(const char *src, int src_h, int src_w, ROTATION_E rotate);Function Introduction: Rotate the incoming image, supporting only 90, 180, 270-degree rotations. Accelerated by VPS. Input and output image data is in the form of nv12 data.

Return Value: Returns the address of the rotated image data if successful, returns nullptr if failed.

Parameters:
| Parameter    | Explanation            |
| ------------ | ---------------------- |
| src          | Address of the input image data |
| src_h        | Height of the input image |
| src_w        | Width of the input image |
| rotate       | Rotation angle          |

### crop&resize&rotate

int hobotcv_imgproc(const cv::Mat &src, cv::Mat &dst, int dst_h, int dst_w, ROTATION_E rotate, const cv::Range &rowRange, const cv::Range &colRange);

Function Introduction: A full-function interface for crop, resize, and rotate. First, crop the specified area in the original image, then resize, and finally rotate. Accelerated by VPS.

Return Value: Returns 0 if successful, non-zero if failed.

Note: dst_h, dst_w are the size after resize. Do not need to consider the width and height after rotation, the interface will handle it automatically. For example, the size after resize is 1920*1080, and dst_w, dst_h are passed as 1920, 1080.

Parameters:
| Parameter    | Explanation                 |
| ------------ | ---------------------------- |
| src          | Original image matrix in nv12 format |
| dst          | Matrix to receive the processed image |
| dst_h        | Height after resize          |
| dst_w        | Width after resize           |
| rotate       | Enumeration of rotation angles, turn off rotate when set to 0 |
| rowRange     | Vertical coordinate range for crop, turn off crop when set to 0 |
| colRange     | Horizontal coordinate range for crop, turn off crop when set to 0 |

std::shared_ptr<ImageInfo> hobotcv_imgproc(const char *src, int src_h, int src_w, int dst_h, int dst_w, ROTATION_E rotate, const cv::Range &rowRange, const cv::Range &colRange);

Function Introduction: A full-function interface for crop, resize, and rotate. First, crop the specified area in the original image, then resize, and finally rotate. Accelerated by VPS. Input and output image data is in the form of nv12 data.

Return Value: Returns a pointer to the processed image data if successful, returns nullptr if failed.

Note: dst_h, dst_w are the size after resize. Do not need to consider the width and height after rotation, the interface will handle it automatically. For example, the size after resize is 1920*1080, and dst_w, dst_h are passed as 1920, 1080.

Parameters:
| Parameter    | Explanation            |
| ------------ | ---------------------- |
| src          | Address of the input image data |
| src_h        | Height of the input image |
| src_w        | Width of the input image |
| dst_h        | Height after resize    |
| dst_w        | Width after resize     |
| rotate       | Enumeration of rotation angles, turn off rotate when set to 0 || rowRange | Vertical coordinate range of crop, closing crop when the range is 0 |
| colRange | Horizontal coordinate range of crop, closing crop when the range is 0 |

### pyramid

int hobotcv_pymscale(const cv::Mat &src, OutputPyramid *output, const PyramidAttr &attr);

Function: Interface for pyramid scaling. Configure reduction and ROI region through parameters attr. The image is reduced to 24 layers (0~23), where layers 0, 4, 8, 12, 16, 20 are basic Base layers, based on the original image for scaling, and the size of each layer in the base layer is half of the previous base layer. The remaining layers are ROI layers, which are reduced based on the Base layer (layers 1, 2, 3 are based on Base0 layer, layers 5, 6, 7 are based on Base4 layer, and so on). Each layer can be individually enabled, and the scaling region and scaling factor can be configured.

Return Value: Returns 0 on success, non-zero on failure.

Note: Maximum input image size is 4096x4096, minimum input image size is 64x64. Maximum output image size is 2048x2048, minimum output image size is 48x32. Due to VPS hardware requirements, Base0 and Base4 layers must be enabled.

ROI layer output calculation formula: targetW = (roi_width - 1) x 64 / (64 + 1) + 1; targetH = (((roi_height / 2 - 1) x 64 / (64 + 1)) + 1) x 2; adjust target width and height to even numbers, if a size of 401 x 401 is obtained, it will be adjusted to 400 x 400

Parameters:
| Parameter | Description |
| -------- | --------------------|
| src | Original image matrix in nv12 format |
| output | Pointer to the output image after pyramid scaling, memory provided by the caller |
| attr | Attributes configuration for pyramid scaling layers |

PyramidAttr: Configuration for pyramid scaling
| Parameter | Description |
| -------------| -----------------------------------|
| timeout | Timeout for obtaining result, in milliseconds |
| ds_info | Configuration information for pyramid reduction layers, including base and roi layers totaling 24 layers |

The range of factor in ds_info is from 0 to 63. For the base layers, factor is 0 to disable the layer, and non-zero to enable the layer. For the roi layers, factor is 0 to disable the layer, and non-zero is the calculation factor for that roi layer. The output size of the layer can be obtained through the roi layer output calculation formula.

OutputPyramid: Data structure for output of pyramid-scaled images
| Parameter | Description |
| -------------| -----------------------------------|
| isSuccess | Whether the interface successfully processes the image, 0: Failure, 1: Success |
| pym_out | Array of information about the output of pyramid scaling, whether there is output for each layer depends on the factor configuration for that layer in PyramidAttr |

int hobotcv_pymscale(const char *src, int src_h, int src_w, OutputPyramid *output, const PyramidAttr &attr);

Function: Interface for pyramid scaling. The function is the same as the previous interface "int hobotcv_pymscale(const cv::Mat &src, OutputPyramid *output, const PyramidAttr &attr)", but the input image is in nv12 format data address.

Return Value: Returns 0 on success, non-zero on failure.

Parameters:
| Parameter | Description |
| -------- | --------------------|
| src | Input image data address |
| src_h | Input image height |
| src_w | Input image width |
| output | Pointer to the output image after pyramid scaling, memory provided by the caller |
| attr | Attributes configuration for pyramid scaling layers |### Border Padding

```cpp
HobotcvImagePtr hobotcv_BorderPadding(const char *src, const int &src_h, const int &src_w, const HobotcvPaddingType type, const PaddingArea &area, const uint8_t value = 0);
```

Function Introduction: Performs padding operation on the input original image, supports specifying padding area and value. Supports three types of padding: HOBOTCV_CONSTANT, HOBOTCV_REPLICATE, and HOBOTCV_REFLECT.
- When type is HOBOTCV_CONSTANT, the value passed into the interface is valid, and the padding is done using the numerical value of 'value'.
- When type is HOBOTCV_REPLICATE, the padding is done using the pixel values from the edges of the original image (e.g., aaaaaaa|abcdefgh|hhhhhhh).
- When type is HOBOTCV_REFLECT, the padding is done by mirroring with the original image boundary as axis (e.g., fedcba|abcdefgh|hgfedcb).
- The height 'h' of the padded image is src_h + area.top + area.bottom, and the width 'w' is src_w + area.left + area.right.
- Upon successful padding, the function returns a pointer to the padded image data.

Return Value: Returns a pointer to the output image data on success, or nullptr on failure.

Parameters:
| Parameter | Description                           |
| --------- | -------------------------------------- |
| src       | Original image data in nv12 format    |
| src_h     | Height of the original image          |
| src_w     | Width of the original image           |
| type      | Padding type                          |
| area      | Padding area, supporting specified top, bottom, left, and right areas |
| value     | Pixel value for padding, range 0~255, default is 0 |

### Gaussian Blur (Accelerated by BPU)

```cpp
int HobotCVGaussianBlurCreate(HobotGaussianBlurParam param, HobotCVGaussianBlurHandle *phandle);
```

Function Introduction: Creates a handle for Gaussian blur.

Return Value: 0 indicates success, <0 indicates failure.

Parameters:
| Parameter | Description           |
| --------- | ---------------------- |
| param     | Gaussian blur parameters |
| --type    | Type of blur           |
| --width   | Blur width             |
| --height  | Blur height            |
| --ksizeX  | Kernel width           |
| --ksizeY  | Kernel height          |
| --sigmaX  | Sigma value for width  |
| --sigmaY  | Sigma value for height  |
| phandle   | Handle returned upon successful creation |

Note: The current version supports the following parameter ranges:
- Blur type: Gaussian blur
- Supported data type: int16- Supported resolution: 320x240.
- Filter kernel: Gaussian 3x3
- sigmax: 0.
- sigmay: 0.

```cpp
int HobotCVGaussianBlurProcess(HobotCVGaussianBlurHandle *phandle, cv::Mat *src, cv::Mat *dst);
```

Function: Create a handle for Gaussian blur.

Return: 0 for success, <0 for failure.

Parameters:

| Parameter | Explanation           |
| --------- | --------------------- |
| phandle   | Handle returned upon successful creation   |
| src       | Original TOF data matrix   |
| dst       | TOF data matrix after filtering  |

int HobotCVGaussianBlurDestroy(HobotCVGaussianBlurHandle *phandle);

Function: Destroy the handle for Gaussian blur.

Return: 0 for success, <0 for failure.

Parameters:

| Parameter | Explanation                         |
| --------- | ------------------------------------|
| phandle   | Handle returned upon successful creation, used for releasing |

### Gaussian Blur (NEON acceleration)

int HobotGaussianBlur(const cv::Mat &src, cv::Mat &dst, cv::Size ksize);

Function: Gaussian blur processing accelerated by NEON.

Return: 0 for success, -2 for non x3 platform, -1 for parameter error.

Parameters:
| Parameter | Explanation                                |
| --------- | -------------------------------------------|
| src       | Input original data matrix, currently only supports CV_16SC1 and CV_16UC1 data types |
| dst       | Output data matrix after Gaussian blur processing |
| ksize     | Size of Gaussian filter template, currently supports only 3x3 and 5x5 size |

### Mean Blur

```cpp
int HobotMeanBlur(const cv::Mat &src, cv::Mat &dst, cv::Size ksize);
```

Function Introduction: Neon accelerated processing for mean filter.

Return Value: 0 indicates success, -2 means not running on x3 platform, -1 indicates parameter error.

Parameters:
| Parameter Name | Explanation |
| -------------- | ----------- |
| src | Input original data matrix, currently only supports CV_16SC1 and CV_16UC1 data types |
| dst | Output data matrix after mean filter processing |
| ksize | Size of mean filter template, currently supports 3x3 and 5x5 sizes |

## hobotcv_benchmark
[Introduction to hobotcv_benchmark](./benchmark/README.md)

## Run
- After successful compilation, copy the generated install path to Horizon X3 development board (ignore the copying step if compiling on X3), and execute the following command to run.

## Running on X3 Ubuntu system

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.bash
# "config" specifies the model used for examples, and local images for the process.
# Copy according to the actual installation path (copy command on X3 Ubuntu is: cp -r install/hobot_cv/lib/hobot_cv/config/ .).

# Launch crop & resize & rotate & pyramid launch file
ros2 launch hobot_cv hobot_cv_crop_resize_rotate_pyramid.launch.py

# Launch test_gaussian_blur launch file
Implement Gaussian blur on images in TOF format through hobot_cv interface.
ros2 launch hobot_cv hobot_cv_gaussian_blur.launch.py

# Launch neon_example launch file
Implement Gaussian blur and mean filter on images in TOF format through hobot_cv interface using neon acceleration.
ros2 launch hobot_cv hobot_cv_neon_blur.launch.py

```

## Running on X3 yocto system

```
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# "config" specifies the model used for examples, and local images for the process.
cp -r install/lib/hobot_cv/config/ .

# Implement crop, resize, and rotate on local images in JPEG format through hobot_cv interface and store the transformed images in JPEG format.
./install/lib/hobot_cv/example
# Run Mode 2:
Implement Gaussian blur on local TOF format images using the Hobot_cv interface, with acceleration by Bou.
ros2 run hobot_cv test_gaussian_blur

# Run Mode 3:
Implement Gaussian blur and mean blur on local TOF format images using the Hobot_cv interface, with acceleration by Neon.
ros2 run hobot_cv neon_example

```

## Running on X86 Ubuntu System

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.bash
# The config file is for the model used in the example and the local images for inference
# Copy according to the actual installation path (copy command for compilation in X3 Ubuntu is cp -r install/hobot_cv/lib/hobot_cv/config/ .).
cp -r install/lib/hobot_cv/config/ .

# Start the resize launch file
ros2 launch hobot_cv hobot_cv_resize.launch.py

```

# Results Analysis

## X3 Results Display

### Crop & Resize & Rotate & Pyramid

```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [example-1]: process started with pid [2840]
[example-1] [INFO] [1655951548.571526792] [example]:
[example-1] source image config/test.jpg is 1920x1080 pixels
[example-1] [INFO] [1655951548.571669584] [example]: resize image to 960x540 pixels, time cost: 280 ms
[example-1] [INFO] [1655951548.642678167] [example]: resize image to 960x540 pixels, time cost: 14 ms
[example-1] [INFO] [1655951548.645212667] [example]: crop image to 960x540 pixels, time cost: 2 ms
[example-1] [INFO] [1655951548.694760625] [example]: crop image to 960x540 pixels, time cost: 1 ms
[example-1] [BPU_PLAT]BPU Platform Version(1.3.1)!
[example-1] [HBRT] set log level as 0. version = 3.13.27
[example-1] [DNN] Runtime version = 1.8.4_(3.13.27 HBRT)
[example-1] [INFO] [1655951548.734399167] [example]: crop image to 960x540 pixels and resize image to 1920x1080 pixels, time cost: 39 ms
[example-1]
[example-1] [INFO] [1655951548.943125584] [example]: crop image to 960x540 pixels and resize image to 1920x1080 pixels, time cost: 15 ms
[example-1]
[example-1] [INFO] [1655951549.077968876] [example]: rotate image 180 , time cost: 134 ms
[example-1][example-1] [INFO] [1655951549.315988376] [example]: second rotate image 180 , time cost: 38 ms
[example-1]
[example-1] [INFO] [1655951549.638380626] [example]: crop image to 960x540 pixels and resize image to 1440x800 pixels and rotate 90, time cost: 322 ms
[example-1]
[example-1] [INFO] [1655951549.764873834] [example]: crop image to 960x540 pixels and resize image to 1440x800 pixels and rotate 90, time cost: 20 ms
[example-1]
[example-1] [INFO] [1655951550.045702293] [example]: pyramid image, time cost: 280 ms
[example-1]
[example-1] [INFO] [1655951550.327614543] [example]: pyramid image, time cost: 19 ms
[INFO] [example-1]: process has finished cleanly [pid 2840]
```

Based on the log, the test program processed the local 1920x1080 resolution image by resize, crop, crop & resize, rotate, crop & resize & rotate, and pyramid methods. The same interface was called twice. The time comparison of the two runs is as follows:

| Image Processing                     | 1st Run Time | 2nd Run Time |
| ------------------------------------ | ------------ | ------------ |
| 1920x1080 resize to 960x540          | 280ms        | 14ms         |
| 1920x1080 crop to 960x540            | 2ms          | 1ms          |
| Crop 960x540 resize to 1920x1080     | 39ms         | 15ms         |
| 1920x1080 rotate 180 degrees         | 134ms        | 38ms         |
| Crop 960x540 resize to 1440x800 and rotate 90 degrees | 322ms | 20ms   |
| Downsizing image with pyramid        | 280ms        | 19ms         |

For the first run, hardware configuration was required, leading to longer processing time. If there are no further changes to the hardware settings, direct processing by the hardware will significantly reduce the processing time.

Original image:
![image](./config/test.jpg)

Resize result:
![image](./imgs/resize.jpg)

Crop result:
![image](./imgs/crop.jpg)

Crop & resize result:
![image](./imgs/cropResize.jpg)

Rotate result:
![image](./imgs/rotate.jpg)

Crop, resize & rotate result:
![image](./imgs/cropResizeRotate.jpg)

Pyramid downsizing effect, each layer is half of the previous layer:![image](./imgs/pym/pym_ds.jpg)

### resize
Launch command: ros2 launch hobot_cv hobot_cv_resize.launch.py
Output:
```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [resize_example-1]: process started with pid [120089]
[resize_example-1] [INFO] [1666364548.561811871] [example]:
[resize_example-1] source image config/test.jpg is 1920x1080 pixels
[resize_example-1] [INFO] [1666364548.561984163] [example]: resize image to 960x540 pixels, time cost: 51 ms
[resize_example-1] [INFO] [1666364548.634829288] [example]: resize image to 960x540 pixels, time cost: 12 ms
[resize_example-1] [INFO] [1666364548.647819913] [example]: nv12 interface resize image to 960x540 pixels, time cost: 12 ms
[INFO] [resize_example-1]: process has finished cleanly [pid 120089]
```

According to the log, example resized the 1920x1080 resolution image three times. The first two times used the cv::Mat interface for input and output images, while the third time used the nv12 data pointer interface for input and output images. The time statistics for the three resize operations are as follows:

| Image Processing         | 1st Run Time | 2nd Run Time | 3rd Run Time |
| ------------------------- | ------------- | ------------- | ------------- |
| 1920x1080 to 960x540      | 51ms         | 12ms         | 12ms         |

The first run took more time due to the need for hardware configuration. Subsequent runs with the same hardware configuration showed significantly reduced processing time. The time taken for resize using the two interfaces did not show significant changes.

### crop
Launch command: ros2 launch hobot_cv hobot_cv_crop.launch.py
Output:
```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [crop_example-1]: process started with pid [117973]
[crop_example-1] [INFO] [1666364496.940625763] [example]: crop image to 960x540 pixels, time cost: 2 ms
[crop_example-1] [INFO] [1666364497.028130347] [example]: crop image to 960x540 pixels, time cost: 2 ms
[crop_example-1] [INFO] [1666364497.032258180] [example]: nv12 interface crop image to 960x540 pixels, time cost: 3 ms
[INFO] [crop_example-1]: process has finished cleanly [pid 117973]
```

According to the log, example cropped the 1920x1080 resolution image to 960x540 three times. The first two times used the cv::Mat interface for input and output images, while the third time used the nv12 data pointer interface for input and output images. The time statistics for the three crop operations are as follows:

| Image Processing         | 1st Run Time | 2nd Run Time | 3rd Run Time |
| ------------------------- | ------------- | ------------- | ------------- |
| 1920x1080 to 960x540      | 2ms          | 2ms          | 3ms          |

Since cropping directly modifies the original image without hardware acceleration, the interface call time showed no significant difference between runs. The third run using the nv12 data pointer interface may have slightly increased time due to memory allocation in the interface.

### rotate
Launch command: ros2 launch hobot_cv hobot_cv_rotate.launch.py
Output:
```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [rotate_example-1]: process started with pid [121764]
[rotate_example-1] [INFO] [1666364588.685647265] [example]: rotate image 180 , time cost: 163 ms
[rotate_example-1]
[INFO] [1666364588.937273432] [example]: second rotate image 180 , time cost: 38 ms
[rotate_example-1]
[INFO] [1666364588.975745807] [example]: nv12 interface rotate image 180 , time cost: 38 ms
[rotate_example-1]
[INFO] [rotate_example-1]: process has finished cleanly [pid 121764]
```

example performed three rotations of 180 degrees on an image with a resolution of 1920x1080. The first two rotations used interfaces with cv::Mat as input and output images, while the third rotation used an interface with nv12 data pointers for input and output images. The time statistics for the three rotations are as follows:

| Image Processing            | First Run Time | Second Run Time | Third Run Time |
| --------------------------- | -------------- | --------------- | -------------- |
| 1920x1080 Rotate 180 degrees| 163ms          | 38ms            | 38ms           |


### padding
example launch command: ros2 launch hobot_cv hobot_cv_padding.launch.py
Output:
```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [padding_example-1]: process started with pid [219943]
[padding_example-1] [INFO] [1666363731.418628584] [example]: 480 x 270 hobot_cv constant padding  top:100 bottom: 100 left: 100 right: 100, time cost: 1 ms
[padding_example-1]
[padding_example-1] [INFO] [1666363731.458502459] [example]: 480 x 270 hobot_cv replicate padding top:100 bottom: 100 left: 100 right: 100, time cost: 1 ms
[padding_example-1]
[padding_example-1] [INFO] [1666363731.493979875] [example]: 480 x 270 hobot_cv reflect padding top:100 bottom: 100 left: 100 right: 100, time cost: 3 ms
[padding_example-1]
[padding_example-1] [INFO] [1666363731.531334959] [example]: 480 x 270 opencv constant padding top:100 bottom: 100 left: 100 right: 100, time cost: 2 ms
[padding_example-1]
[padding_example-1] [INFO] [1666363731.562557334] [example]: 480 x 270 opencv replicate padding top:100 bottom: 100 left: 100 right: 100, time cost: 1 ms
[padding_example-1]
[padding_example-1] [INFO] [1666363731.594738500] [example]: 480 x 270 opencv reflect padding top:100 bottom: 100 left: 100 right: 100, time cost: 1 ms
[padding_example-1]
[INFO] [padding_example-1]: process has finished cleanly [pid 219943]
```

According to the log, the test program completed padding the top, bottom, left, and right regions of a local 480x270 resolution image with a length of 100 each. The time comparisons between hobot_cv and opencv padding methods are as follows:
| Padding Method    | hobot_cv Time | opencv Time |
| ----------------- | ------------- | ----------- |
| CONSTANT          | 1ms           | 2ms         |
| REPLICATE         | 1ms           | 1ms         |
| REFLECT           | 3ms           | 1ms         |

Original Image:
![image](./config/480x270.jpg)

HOBOTCV_CONSTANT Padding:
![image](./imgs/cv_constant_padding.jpg)

HOBOTCV_REPLICATE Padding:
![image](./imgs/cv_replicate_padding.jpg)

HOBOTCV_REFLECT fill display:
![image](./imgs/cv_reflect_padding.jpg)

### Gaussian Blurring Accelerated by BPU

```
Output:
===================
image name: images/frame1_4.png
infe cost time: 1314
guss_time cost time: 2685
hobotcv save rate: 0.510615

analysis_result start
---------GaussianBlur
out_filter type: 2, cols: 320, rows: 240, channel: 1
cls_filter type: 2, cols: 320, rows: 240, channel: 1
out_filter min value: 96, max: 2363
out_filter min, x: 319, y: 115
out_filter max, x: 147, y: 239
cls_filter min value: 96, max: 2364
cls_filter min, x: 319, y: 115
cls_filter max, x: 147, y: 239

diff diff diff
mat_diff min value: 0, max: 2
mat_diff min, x: 2, y: 0
mat_diff max, x: 110, y: 14

error sum: 8.46524e+06, max: 2, mean error: 0.439232
analysis_result, time_used_ms_end: 2
analysis_result end

Where:

infe cost time: 1314 // Represents the time of Gaussian blurring accelerated by HOBOTCV, taking 1314 microseconds.

guss_time cost time: 2685 // Represents the time of Gaussian blurring using OpenCV, taking 2685 microseconds.

hobotcv save rate = (guss_time cost time - infe cost time) / guss_time cost time = 0.510615

From the comparison above, the performance is improved by 50% with HOBOTCV acceleration.

error sum: 8.46524e+06, max: 2, mean error: 0.439232 // The total error for a single image is 8.46524e+06, the maximum error for a single pixel is 2, and the average error is 0.439232.

Average error = sum / (width * height) = 8.46524e+06 / (320 * 240)### Gaussian Filtering and Mean Filtering with Neon Acceleration
Perform Gaussian filtering and mean filtering on images in local tof format using the hobot_cv interface, and log the comparison of the acceleration with neon and opencv processing efficiency.

[neon_example-1] ===================
[neon_example-1] image name: config/tof_images/frame1_4.png
[neon_example-1] hobotcv mean cost time: 674
[neon_example-1] opencv mean cost time: 1025
[neon_example-1] hobotcv mean save rate: 0.342439
[neon_example-1]
[neon_example-1] analyse_result start
[neon_example-1] ---------Mean_Blur
[neon_example-1] error sum: 8.43744e+06, max: 1, mean_error: 0.430833
[neon_example-1]
[neon_example-1] hobotcv gaussian cost time: 603
[neon_example-1] opencv gaussian cost time: 2545
[neon_example-1] hobotcv gaussian save rate: 0.763065
[neon_example-1]
[neon_example-1] analyse_result start
[neon_example-1] ---------Gaussian_Blur
[neon_example-1] error sum: 9.13206e+06, max: 1, mean_error: 0.466302
[neon_example-1]
[neon_example-1] -------------------------

hobotcv mean cost time: 674 // The time consumed by the hobotcv mean filtering neon acceleration interface is 674 microseconds.
opencv mean cost time: 1025 // Indicates that opencv's mean filtering takes 1025 microseconds.
hobotcv mean save rate = (opencv cost time - hobotcv cost time) / opencv cost time = 0.342439

hobotcv gaussian cost time: 603 // The time consumed by the hobotcv Gaussian filtering neon acceleration interface is 603 microseconds.
opencv gaussian cost time: 2545 // Indicates that opencv's Gaussian filtering takes 2545 microseconds.
hobotcv gaussian save rate = (opencv cost time - hobotcv cost time) / opencv cost time = 0.763065

According to the comparison results above, after acceleration by hobotcv, the performance of mean filtering improves by 34%, and Gaussian filtering improves by 76%.

error sum: 8.43744e+06, max: 1, mean_error: 0.430833 // The total error of mean filtering in a single image is 8.43744e+06, the maximum error per pixel is 1, and the mean error is 0.430833.
Mean filtering average error = sum / (width x height) = 8.43744e+06 / (320 x 240)

error sum: 9.13206e+06, max: 1, mean_error: 0.466302 // The total error of Gaussian filtering in a single image is 9.13206e+06, the maximum error per pixel is 1, and the mean error is 0.466302.
Gaussian filtering average error = sum / (width x height) = 9.13206e+06 / (320 x 240)
```
