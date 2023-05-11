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
        "rotation", default_value=TextSubstitution(text="180")
    )
    dst_width_launch_arg = DeclareLaunchArgument(
        "dst_width", default_value=TextSubstitution(text="960")
    )
    dst_height_launch_arg = DeclareLaunchArgument(
        "dst_height", default_value=TextSubstitution(text="540")
    )
    process_type_launch_arg = DeclareLaunchArgument(
        "process_type", default_value=TextSubstitution(text="0")
    )
    img_fmt_launch_arg = DeclareLaunchArgument(
        "img_fmt", default_value=TextSubstitution(text="0")
    )
    speedup_type_launch_arg = DeclareLaunchArgument(
        "speedup_type", default_value=TextSubstitution(text="0")
    )

    static_cycle_launch_arg = DeclareLaunchArgument(
        "static_cycle", default_value=TextSubstitution(text="1000")
    )

    hobotcv_benchmark_node = Node(
        package='hobot_cv',
        executable='hobotcv_benchmark',
        output='screen',
        parameters=[
            {"image_file": LaunchConfiguration('image_file')},
            {"rotation": LaunchConfiguration('rotation')},
            {"dst_width": LaunchConfiguration('dst_width')},
            {"dst_height": LaunchConfiguration('dst_height')},
            {"process_type": LaunchConfiguration('process_type')},
            {"img_fmt": LaunchConfiguration('img_fmt')},
            {"speedup_type": LaunchConfiguration('speedup_type')},
            {"static_cycle": LaunchConfiguration('static_cycle')}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        img_file_launch_arg,
        rotation_launch_arg,
        dst_width_launch_arg,
        dst_height_launch_arg,
        process_type_launch_arg,
        img_fmt_launch_arg,
        speedup_type_launch_arg,
        static_cycle_launch_arg,
        hobotcv_benchmark_node
    ])
