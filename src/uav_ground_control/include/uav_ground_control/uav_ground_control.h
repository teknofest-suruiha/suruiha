#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <string>

class UavGroundControl {

public: UavGroundControl(ros::NodeHandle& _node);
public: virtual ~UavGroundControl();
public: void spinOnce();
public: void init(std::string controlTopicName, std::string poseTopicName);
private: void poseCallback(const geometry_msgs::Pose::ConstPtr& pose);

private:
	ros::NodeHandle node;
	std::vector<ros::Subscriber> poseSubs;
	std::vector<ros::Publisher> controlPubs;
};
