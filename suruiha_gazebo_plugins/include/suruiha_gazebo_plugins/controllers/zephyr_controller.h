#ifndef ZEPHYR_CONTROLLER_H
#define ZEPHYR_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>
#include <suruiha_gazebo_plugins/controllers/joint_control.h>

namespace gazebo
{
	class ZephyrController : public ModelPlugin
    {

		public: ZephyrController();
		public: virtual ~ZephyrController();

		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
		protected: virtual void UpdateStates();
		private: event::ConnectionPtr update_connection_;

		private: physics::WorldPtr world_;
		private: physics::ModelPtr model_;
//		private: physics::LinkPtr bodyLink_;
		
		private: std::string robot_namespace_;
//		private: std::string control_topic_name_;
//		private: std::string pose_topic_name_;
//		private: std::string user_command_topic_name_;
		private: ros::NodeHandle* rosnode_;
        private: ros::Subscriber control_twist_sub_;
//        private: ros::Subscriber user_command_sub_;
        private: ros::Publisher pose_pub_;

        private: double targetThrottle;
        private: double targetPitch;
        private: double targetRoll;

        private: std::vector<JointControl*> joints_;
        private: void SetPIDParams(JointControl* jointControl, sdf::ElementPtr _sdf);
	    private: void SetControl(const geometry_msgs::Twist::ConstPtr& controlTwist);
//	    private: void ProcessUserCommand(const std_msgs::String::ConstPtr& user_command);
	    private: void CalculateJoints(double targetThrottle, double targetPitch, double targetRoll, common::Time dt);

		private: boost::mutex update_mutex_;
		private: ros::CallbackQueue queue_;
		private: boost::thread callback_queue_thread_;
	    private: void QueueThread();

        private: common::Time lastUpdateTime;

        private: common::Time lastPosePublishTime;
        private: int poseUpdateRate;

	};
}

#endif
