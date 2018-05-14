#include <uav_ground_control/uav_ground_control.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>

UavGroundControl::UavGroundControl(ros::NodeHandle& _node):node(_node) {

}

UavGroundControl::~UavGroundControl() {

}

void UavGroundControl::spinOnce() {

}

void UavGroundControl::init(std::string controlTopicName, std::string poseTopicName) {
	controlPubs.push_back(node.advertise<geometry_msgs::Twist>(controlTopicName.c_str(), 10));
	poseSubs.push_back(node.subscribe(poseTopicName.c_str(), 10, &UavGroundControl::poseCallback, this));
}

void UavGroundControl::poseCallback(const geometry_msgs::Pose::ConstPtr& pose) {
	//tf::Quaternion orientation(pose->pose.pose.orientation.x, pose->pose.pose.orientation.y,
	//		pose->pose.pose.orientation.z, pose->pose.pose.orientation.w);
	//tf::Matrix3x3 quatMat(orientation);
	//tfScalar yaw, pitch, roll;

	// for our model roll is pitch of the plane
	// for our model pitch is roll of the plane
	//quatMat.getEulerYPR(yaw, pitch, roll);
	//ROS_INFO("yaw:%f pitch:%f roll:%f", yaw, pitch, roll);
//	roll = -roll;

//	const double targetPitch = 0.05;
//	roll = targetPitch - roll;

//	double left_flap = (pose->pose.pose.orientation.x - pose->pose.pose.orientation.y);
//	double right_flap = (pose->pose.pose.orientation.x + pose->pose.pose.orientation.y);

//	ROS_INFO("left_flap:%f right_flap:%f", left_flap, right_flap);

//	const double propeller_speed = 400.0f;

	geometry_msgs::Twist control;
	control.linear.x = 400;
	
	control.angular.y = -0.05;
	control.angular.x = 0.0;

	controlPubs[0].publish(control);
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "uav_ground_control");
  ros::NodeHandle n;

  int rate;
  std::string controlTopicName;
  std::string poseTopicName;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can
  // be run simultaneously while using different parameters.
  // Parameters defined in the .cfg file do not need to be initialized here
  // as the dynamic_reconfigure::Server does this for you.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(10));
  private_node_handle_.param("control_topic", controlTopicName, std::string("zephyr_control"));
  private_node_handle_.param("pose_topic", poseTopicName, std::string("zephyr_pose"));

  UavGroundControl uavControl(n);

  // create publishers and subscribers
  uavControl.init(controlTopicName, poseTopicName);

  // Tell ROS how fast to run this node.
  ros::Rate r(rate);

  // Main loop.
  while (n.ok())
  {
	  uavControl.spinOnce();
	  ros::spinOnce();
	  r.sleep();
  }

  return 0;
} // end main()
