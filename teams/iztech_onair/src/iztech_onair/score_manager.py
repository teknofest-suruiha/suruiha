from suruiha_gazebo_plugins.msg import UAVScore
import rospy


class ScoreManager:
    def __init__(self):
        rospy.Subscriber('/score', UAVScore, self.message_received)
        self.last_score = UAVScore()

    def message_received(self, score_msg):
        self.last_score = score_msg

    def get_last_score(self):
        return self.last_score

    def step(self):
        print('area_score:' + str(self.last_score.area_score))
        print('terorist_detection_score:' + str(self.last_score.detection_score))
        print('terorist_tracking_score:' + str(self.last_score.tracking_score))
        print('total_score:' + str(self.last_score.total_score))

