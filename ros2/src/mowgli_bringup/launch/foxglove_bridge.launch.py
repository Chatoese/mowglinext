# Copyright 2026 Mowgli Project
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


"""
foxglove_bridge.launch.py

Starts the foxglove_bridge WebSocket server for Foxglove Studio.

Foxglove Bridge provides a high-performance binary WebSocket protocol,
replacing the JSON-based rosbridge_server used on Humble.

Connect Foxglove Studio to: ws://<host>:8765
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Declared arguments
    # ------------------------------------------------------------------
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="8765",
        description="Port number for the Foxglove Bridge WebSocket server.",
    )

    send_buffer_limit_arg = DeclareLaunchArgument(
        "send_buffer_limit",
        # 1 MB (was 10 MB): when a client falls behind on the WebSocket,
        # the bridge buffers up to this many bytes per client before
        # dropping old messages. 10 MB at the typical visualisation
        # bandwidth (~250 KB/s) is ~40 s of backlog, which manifests as
        # multi-second pose/scan lag in the viewer even though the ROS2
        # side is real-time. 1 MB caps backlog at ~4 s; combined with
        # the lower /tf rate after the recent tuning (99 Hz instead of
        # 190 Hz), the buffer should rarely fill at all on a healthy
        # link, and when it does the latest pose/scan is preferred over
        # buffered history.
        default_value="1000000",
        description="Maximum bytes buffered per client before dropping messages.",
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    port = LaunchConfiguration("port")
    send_buffer_limit = LaunchConfiguration("send_buffer_limit")

    # ------------------------------------------------------------------
    # foxglove_bridge node
    # ------------------------------------------------------------------
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": port,
                "address": "0.0.0.0",
                "send_buffer_limit": send_buffer_limit,
                # 1 (was 0 = one asio thread per core, i.e. 4 on the Pi).
                # The bridge serves one GUI backend plus an occasional
                # Foxglove Studio session at ~250 KB/s — a single handler
                # thread is plenty, and 3 fewer threads means measurably
                # less scheduler/context-switch overhead on the 4-core Pi
                # (the bridge sat at ~5 % average CPU, #4 in the container).
                "num_threads": 1,
                # Capability trim: the mowgli GUI backend uses subscribe,
                # clientPublish (teleop), services (map editing) and
                # parameters/parametersSubscribe (Settings page) — keep all
                # of those. Dropped: connectionGraph (only Foxglove Studio's
                # topology panel; subscribing it makes the bridge poll the
                # FULL ROS graph every second) and assets (URDF fetch — the
                # GUI's chassis preview reads the robotDescription TOPIC,
                # not the asset capability).
                "capabilities": [
                    "clientPublish",
                    "parameters",
                    "parametersSubscribe",
                    "services",
                ],
            },
        ],
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            port_arg,
            send_buffer_limit_arg,
            foxglove_bridge_node,
        ]
    )
