import rospy
from suruiha_gazebo_plugins.msg import UAVTracking
from std_msgs.msg import String


class TerroristDetector:
    def __init__(self, sensor_manager):
        self.terrorist_tracking_publisher = rospy.Publisher('/terrorist_tracking', UAVTracking, queue_size=1)
        self.terrorist_detection_publisher = rospy.Publisher('/terrorist_detection', String, queue_size=1)
        self.sensor_manager = sensor_manager

    def step(self, perception=None):
        if perception is None:
            perception = self.sensor_manager.get_last_perception()
        trackingMsg = UAVTracking()
        for i in range(len(perception.types)):
            if 'terrorist' in perception.names[i]:
                trackingMsg.names.append(perception.names[i])
                trackingMsg.poses.append(perception.poses[i])
            else:
                print("ELSE:", str(perception.names[i]))


            self.terrorist_tracking_publisher.publish(trackingMsg)
            print(trackingMsg)

        if len(trackingMsg.names) > 0:
            detectionMsg = String()
            detectionMsg.data = 'building_1'
            #self.terrorist_detection_publisher.publish(detectionMsg)

    def publish(self, uav_map, perception=None):
        if perception is None:
            perception = self.sensor_manager.get_last_perception()


        for index, pose in enumerate(perception.poses):
            name = perception.names[index]
            if "terrorist" in name:
                tracking_msg = UAVTracking()
                tracking_msg.names.append(perception.names[index])
                tracking_msg.poses.append(perception.poses[index])
                self.terrorist_tracking_publisher.publish(tracking_msg)
                print("TRACKING", str(tracking_msg))
            # TODO: Bina tespit edilince duzenleyerek aktive et
            '''
            elif "building" in name:
                # if name not in uav_map.detected_objects:
                detectionMsg = String()
                detectionMsg.data = name
                self.terrorist_detection_publisher.publish(detectionMsg)
                print("DETECTED", str(detectionMsg))
            '''


