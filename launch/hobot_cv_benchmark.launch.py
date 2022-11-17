# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from argparse import Action

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    img_file_launch_arg = DeclareLaunchArgument(
        "image_file", default_value=TextSubstitution(text="config/test.jpg")
    )
    rotation_launch_arg = DeclareLaunchArgument(
        "rotation",default_value=TextSubstitution(text="180")
    )
    dst_width_launch_arg = DeclareLaunchArgument(
        "dst_width",default_value=TextSubstitution(text="960")
    )
    dst_height_launch_arg = DeclareLaunchArgument(
        "dst_height",default_value=TextSubstitution(text="540")
    )
    cv_type_launch_arg = DeclareLaunchArgument(
        "cv_type",default_value=TextSubstitution(text="resize")
    )
    interface_type_launch_arg = DeclareLaunchArgument(
        "interface_type",default_value=TextSubstitution(text="2")
    )
    speed_type_launch_arg = DeclareLaunchArgument(
        "speed_type",default_value=TextSubstitution(text="vps")
    )


    return LaunchDescription([
        img_file_launch_arg,
        rotation_launch_arg,
        dst_width_launch_arg,
        dst_height_launch_arg,
        cv_type_launch_arg,
        interface_type_launch_arg,
        speed_type_launch_arg,
        Node(
            package='hobot_cv',
            executable='hobotcv_benchmark',
            output='screen',
            parameters=[
                {"image_file": LaunchConfiguration('image_file')},
                {"rotation": LaunchConfiguration('rotation')},
                {"dst_width": LaunchConfiguration('dst_width')},
                {"dst_height": LaunchConfiguration('dst_height')},
                {"cv_type": LaunchConfiguration('cv_type')},
                {"interface_type": LaunchConfiguration('interface_type')},
                {"speed_type": LaunchConfiguration('speed_type')}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
