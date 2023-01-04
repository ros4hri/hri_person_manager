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
