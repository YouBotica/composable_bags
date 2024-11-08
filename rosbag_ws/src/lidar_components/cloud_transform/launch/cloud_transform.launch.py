# Copyright 2018 Open Source Robotics Foundation, Inc.
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

# FIXME: This file is for unit-testing purpose ONLY.
# DO NOT INCLUDE this file in the launch procedure of the whole stack.
# Modification of this file will NOT be reflected in the launch of the whole stack.

from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description="Use simulation clock if True"
        ),

        launch_ros.actions.Node(
            package="cloud_transform",
            executable="cloud_transform",
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('/luminar_lidar_front/points', 'luminar_front_points'),
                ('/luminar_lidar_left/points', 'luminar_left_points'),
                ('/luminar_lidar_right/points', 'luminar_right_points'),
            ],
        )
])
