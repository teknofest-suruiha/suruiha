from suruiha_gazebo_plugins.msg import UAVScore
import rospy


class ScoreManager:
    def __init__(self):
        rospy.Subscriber('/score', UAVScore, self.message_received)
        self.current_score = UAVScore()

    def message_received(self, score_msg):
        self.current_score = score_msg

    def get_current_score(self):
        return self.current_score