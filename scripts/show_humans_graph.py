import graphviz
import rospy
from std_msgs.msg import String


def callback(msg):
    src = graphviz.Source(msg.data)
    src.render(outfile="/tmp/graph.pdf", format="pdf")


if __name__ == "__main__":
    rospy.init_node("human_graph_viewer", anonymous=True)
    rospy.Subscriber("/humans/graph", String, callback)
    rospy.spin()
