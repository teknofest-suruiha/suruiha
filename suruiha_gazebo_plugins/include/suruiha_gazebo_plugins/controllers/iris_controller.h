#ifndef ZEPHYR_CONTROLLER_H
#define ZEPHYR_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <ros/callback_queue.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>
#include <suruiha_gazebo_plugins/controllers/rotor_control.h>
#include <vector>
#include <suruiha_gazebo_plugins/uav_sensor/uav_sensor.h>
#include <suruiha_gazebo_plugins/battery/battery_manager.h>
#include <gazebo/sensors/Sensor.hh>

namespace gazebo
{
	class IrisController : public ModelPlugin
    {
		public: IrisController();
		public: virtual ~IrisController();

		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
		protected: virtual void UpdateStates();
		private: event::ConnectionPtr update_connection_;

		private: physics::WorldPtr world_;
		private: physics::ModelPtr model_;
		
//		private: std::string robot_namespace_;
//		private: std::string user_command_topic_name_;
        private: std::string modelName;
		private: ros::NodeHandle* rosnode_;
        private: ros::Subscriber control_twist_sub_;
//        private: ros::Subscriber user_command_sub_;
        private: ros::Publisher pose_pub_;
//	    private: sensor_msgs::JointState last_joint_states_;
        private: double targetThrottle;
        private: double targetPitch;
        private: double targetRoll;
        private: double targetYaw;

        private: std::vector<RotorControl*> rotors_;
	    private: void SetControl(const geometry_msgs::Twist::ConstPtr& control);
//	    private: void ProcessUserCommand(const std_msgs::String::ConstPtr& user_command);
	    private: void CalculateRotors(double targetThrottle, double targetPitch, double targetRoll,
	    		double targetYaw, common::Time dt);
        protected: void SetZeroForceToJoints();

		private: UAVSensor uavSensor;
		private: common::Time lastSensorTime;
		private: int sensorUpdateReate;
		private: ros::Publisher sensor_pub_;

		private: boost::mutex update_mutex_;
		private: ros::CallbackQueue queue_;
		private: boost::thread callback_queue_thread_;
	    private: void QueueThread();

        private: common::Time lastUpdateTime;

        private: common::Time lastPosePublishTime;
        private: int poseUpdateRate;

        // communcation between air traffic controller and fligh controller
        private: transport::NodePtr node;
        private: transport::SubscriberPtr airControlSubPtr;
        protected: void OnAirControlMsg(ConstAnyPtr& airControlMsg);
		private: transport::SubscriberPtr batteryReplaceSubPtr;
		protected: void OnBatteryReplaceMsg(ConstAnyPtr& batteryReplaceMsg);
        protected: bool isActive;

        // battery manager
        protected: BatteryManager battery;

		protected: sensors::SensorPtr logicalCameraSensorPtr;
		protected: sensors::SensorPtr cameraSensorPtr;

	};
}

#endif
