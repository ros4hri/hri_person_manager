# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
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

import argparse
import graphviz
import rclpy
from std_msgs.msg import String


filename = "/tmp/graph"


def callback(msg):
    src = graphviz.Source(msg.data)
    src.render(filename=filename, format="pdf")


def main():
    global filename

    parser = argparse.ArgumentParser(description="Save person graph")
    parser.add_argument(
        "filename", nargs="?", default=filename, help="Recorded graph filename.")
    args = parser.parse_args()
    filename = args.filename

    rclpy.init()
    node = rclpy.create_node("human_graph_viewer")
    node.create_subscription(String, "/humans/graph", callback, 1)

    node.get_logger().info(
        f"Ready! Open {args.filename}.pdf in your favorite PDF viewer to display the graph.")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
