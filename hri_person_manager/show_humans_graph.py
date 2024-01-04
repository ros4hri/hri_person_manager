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
import graphviz
import rospy
from std_msgs.msg import String

PATH = "/tmp/graph.pdf"


def callback(msg):
    src = graphviz.Source(msg.data)
    src.render(outfile=PATH, format="pdf")


if __name__ == "__main__":
    rospy.init_node("human_graph_viewer", anonymous=True)
    rospy.Subscriber("/humans/graph", String, callback)

    rospy.loginfo(
        "Ready! Open %s in your favorite PDF viewer to display the graph." % PATH
    )
    rospy.spin()
