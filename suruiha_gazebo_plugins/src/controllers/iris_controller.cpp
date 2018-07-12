/**
 *  \author Okan Asik
 *  \desc   Gazebo Plugin to control Zephyr fixed wing plane
 */

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <suruiha_gazebo_plugins/controllers/iris_controller.h>
#include <gazebo/common/Plugin.hh>
#include <ignition/math.hh>
#include <sdf/sdf.hh>
#include <suruiha_gazebo_plugins/util/util.h>

namespace gazebo {

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(IrisController);

    // define class static const variables
//    const unsigned int ZephyrController::PROPELLER_JOINT = 0;
//    const unsigned int ZephyrController::FLAP_LEFT_JOINT = 1;
//    const unsigned int ZephyrController::FLAP_RIGHT_JOINT = 2;
//
//    std::string ZephyrController::PROPELLER_JOINT_STR = "propeller_joint";
//    std::string ZephyrController::FLAP_LEFT_JOINT_STR = "flap_left_joint";
//    std::string ZephyrController::FLAP_RIGHT_JOINT_STR = "flap_right_joint";

    IrisController::IrisController() {
        lastUpdateTime = 0;
        targetThrottle = 0.0;
        targetPitch = 0.0;
        targetRoll = 0.0;
        targetYaw = 0.0;
        poseUpdateRate = 100;
    }

    IrisController::~IrisController() {
        this->update_connection_.reset();
        this->rosnode_->shutdown();
        this->queue_.clear();
        this->queue_.disable();
        this->callback_queue_thread_.join();
        delete this->rosnode_;
        for (unsigned i = 0; i < rotors_.size(); i++) {
        	delete rotors_[i];
        }
        rotors_.clear();
    }

    void IrisController::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        this->model_ = _parent;
        this->world_ = this->model_->GetWorld();

        std::string model_name = _sdf->GetParent()->GetAttribute("name")->GetAsString();
        std::string pose_topic = model_name + "_pose";
//        std::cout <<"pose_topic:" << pose_topic << std::endl;
        std::string control_topic = model_name + "_control";
//        std::cout <<"control_topic:" << control_topic << std::endl;
        poseUpdateRate = _sdf->Get<int>("poseUpdateRate");

         if (_sdf->HasElement("rotor"))
         {
           sdf::ElementPtr rotorSDF = _sdf->GetElement("rotor");

           while (rotorSDF)
           {
             RotorControl* rotor = new RotorControl();

             if (rotorSDF->HasAttribute("id"))
             {
               rotor->id = rotorSDF->GetAttribute("id")->Get(rotor->id);
             }
             else
             {
               rotor->id = rotors_.size();
               gzwarn << "id attribute not specified, use order parsed ["
                      << rotor->id << "].\n";
             }

             if (rotorSDF->HasElement("jointName"))
             {
               rotor->jointName = rotorSDF->Get<std::string>("jointName");
             }
             else
             {
               gzerr << "Please specify a jointName,"
                 << " where the rotor is attached.\n";
             }

             // Get the pointer to the joint.
             rotor->joint = model_->GetJoint(rotor->jointName);
             if (rotor->joint == nullptr)
             {
               gzerr << "Couldn't find specified joint ["
                   << rotor->jointName << "]. This plugin will not run.\n";
               return;
             }

             if (rotorSDF->HasElement("turningDirection"))
             {
               std::string turningDirection = rotorSDF->Get<std::string>(
                   "turningDirection");
               // special cases mimic from rotors_gazebo_plugins
               if (turningDirection == "cw")
                 rotor->multiplier = -1;
               else if (turningDirection == "ccw")
                 rotor->multiplier = 1;
               else
               {
                 gzdbg << "not string, check turningDirection as float\n";
                 rotor->multiplier = rotorSDF->Get<double>("turningDirection");
               }
             }
             else
             {
               rotor->multiplier = 1;
               gzerr << "Please specify a turning"
                 << " direction multiplier ('cw' or 'ccw'). Default 'ccw'.\n";
             }

             Util::GetSdfParam(rotorSDF, "rotorVelocitySlowdownSim",
                 rotor->rotorVelocitySlowdownSim, 1);

             if (ignition::math::equal(rotor->rotorVelocitySlowdownSim, 0.0))
             {
               gzerr << "rotor for joint [" << rotor->jointName
                     << "] rotorVelocitySlowdownSim is zero,"
                     << " aborting plugin.\n";
               return;
             }

             Util::GetSdfParam(rotorSDF, "frequencyCutoff",
                 rotor->frequencyCutoff, rotor->frequencyCutoff);
             Util::GetSdfParam(rotorSDF, "samplingRate",
                 rotor->samplingRate, rotor->samplingRate);

             // use ignition::math::Filter
             rotor->velocityFilter.Fc(rotor->frequencyCutoff, rotor->samplingRate);

             // initialize filter to zero value
             rotor->velocityFilter.Set(0.0);

             // note to use this
             // rotorVelocityFiltered = velocityFilter.Process(rotorVelocityRaw);

             // Overload the PID parameters if they are available.
             double param;
             Util::GetSdfParam(rotorSDF, "vel_p_gain", param, rotor->pid.GetPGain());
             rotor->pid.SetPGain(param);

             Util::GetSdfParam(rotorSDF, "vel_i_gain", param, rotor->pid.GetIGain());
             rotor->pid.SetIGain(param);

             Util::GetSdfParam(rotorSDF, "vel_d_gain", param,  rotor->pid.GetDGain());
             rotor->pid.SetDGain(param);

             Util::GetSdfParam(rotorSDF, "vel_i_max", param, rotor->pid.GetIMax());
             rotor->pid.SetIMax(param);

             Util::GetSdfParam(rotorSDF, "vel_i_min", param, rotor->pid.GetIMin());
             rotor->pid.SetIMin(param);

             Util::GetSdfParam(rotorSDF, "vel_cmd_max", param,
                 rotor->pid.GetCmdMax());
             rotor->pid.SetCmdMax(param);

             Util::GetSdfParam(rotorSDF, "vel_cmd_min", param,
                 rotor->pid.GetCmdMin());
             rotor->pid.SetCmdMin(param);

             // set pid initial command
             rotor->pid.SetCmd(0.0);

             rotors_.push_back(rotor);
             rotorSDF = rotorSDF->GetNextElement("rotor");
           }
         }


        // Make sure the ROS node for Gazebo has already been initalized
        if (!ros::isInitialized()) {
            ROS_FATAL_STREAM_NAMED("template", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                    << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);
            ros::SubscribeOptions joints_so =
                    ros::SubscribeOptions::create<geometry_msgs::Twist>(
                            control_topic, 100, boost::bind(
                                    &IrisController::SetControl, this, _1),
                            ros::VoidPtr(), &this->queue_);
            this->control_twist_sub_ = this->rosnode_->subscribe(joints_so);

        // subscribe for user commands
//        ros::SubscribeOptions user_command_so =
//                            ros::SubscribeOptions::create<std_msgs::String>(
//                                    this->user_command_topic_name_, 100, boost::bind(
//                                            &ZephyrController::ProcessUserCommand, this, _1),
//                                    ros::VoidPtr(), &this->queue_);
//        this->user_command_sub_ = this->rosnode_->subscribe(user_command_so);

        // start custom queue for controller plugin ros topics
        this->callback_queue_thread_ =
                boost::thread(boost::bind(&IrisController::QueueThread, this));

        // create the publisher
        this->pose_pub_ = this->rosnode_->advertise<geometry_msgs::Pose>(pose_topic, 1);

        // New Mechanism for Updating every World Cycle
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&IrisController::UpdateStates, this));
    }

    void IrisController::UpdateStates() {
    	boost::mutex::scoped_lock lock(this->update_mutex_);
    	common::Time currTime = this->world_->SimTime();

    	if (this->pose_pub_.getNumSubscribers() > 0) {
    		double dt_ = (currTime - lastPosePublishTime).Double()*1000; // miliseconds
    		if (dt_ > poseUpdateRate) {
    			ignition::math::Pose3d pose = this->model_->WorldPose();
    			geometry_msgs::Pose poseMsg;
    			poseMsg.position.x = pose.Pos().X();
    			poseMsg.position.y = pose.Pos().Y();
    			poseMsg.position.z = pose.Pos().Z();
    			poseMsg.orientation.x = pose.Rot().X();
    			poseMsg.orientation.y = pose.Rot().Y();
    			poseMsg.orientation.z = pose.Rot().Z();
    			poseMsg.orientation.w = pose.Rot().W();
    			this->pose_pub_.publish(poseMsg);
    			lastPosePublishTime = currTime;
    		}
    	}

    	if (control_twist_sub_.getNumPublishers() > 0) {
    		double dt_ = (currTime - lastUpdateTime).Double();

			// get joint values from planner and set joints
			CalculateRotors(targetThrottle, targetPitch, targetRoll, targetYaw, dt_);
    	}

        this->lastUpdateTime = currTime;
    }

    void IrisController::CalculateRotors(double targetThrottle, double targetPitch, double targetRoll,
    		double targetYaw, common::Time dt) {
	ignition::math::Pose3d pose = model_->WorldPose();
	double pitchFactor = pose.Rot().Euler().Y() + 0.041 - targetPitch;
	double frontPitchRotors = 1.0 - pitchFactor;
	double rearPitchRotors = 1.0 + pitchFactor;

	double rollFactor = pose.Rot().Euler().X() + 0.000 - targetRoll;
	double frontRollRotors = 1.0 - rollFactor;
	double rearRollRotors = 1.0 + rollFactor;

	double yawFactor = pose.Rot().Euler().Z() - targetYaw; //Normalize(pose.Rot().Euler().Z());
	//if (yawFactor > 1.0) yawFactor = 0.5;
	//if (yawFactor < -1.0) yawFactor = -0.5;
	double frontYawRotors = 1.0 - yawFactor;
	double rearYawRotors = 1.0 + yawFactor;

	for (unsigned i = 0; i < rotors_.size(); ++i) {
		rotors_[i]->cmd = targetThrottle;
	}

	rotors_[3]->cmd = rotors_[3]->cmd * frontPitchRotors;
	rotors_[1]->cmd = rotors_[1]->cmd * frontPitchRotors;

	rotors_[0]->cmd = rotors_[0]->cmd * rearPitchRotors;
	rotors_[2]->cmd = rotors_[2]->cmd * rearPitchRotors;

//	for (unsigned i = 0; i < rotors_.size(); ++i) {
//		std::cout << "pitch control:" << rotors_[i]->cmd << std::endl;
//	}

	rotors_[3]->cmd = rotors_[3]->cmd * rearRollRotors;
	rotors_[0]->cmd = rotors_[0]->cmd * rearRollRotors;

	rotors_[1]->cmd = rotors_[1]->cmd * frontRollRotors;
	rotors_[2]->cmd = rotors_[2]->cmd * frontRollRotors;

//	for (unsigned i = 0; i < rotors_.size(); ++i) {
//		std::cout << "pitch and roll control:" << rotors_[i]->cmd << std::endl;
//	}

	rotors_[3]->cmd = rotors_[3]->cmd * frontYawRotors;
	rotors_[2]->cmd = rotors_[2]->cmd * frontYawRotors;

	rotors_[1]->cmd = rotors_[1]->cmd * rearYawRotors;
	rotors_[0]->cmd = rotors_[0]->cmd * rearYawRotors;

//	for (unsigned i = 0; i < rotors_.size(); ++i) {
//		std::cout << "pitch and roll and yaw control:" << rotors_[i]->cmd
//				<< std::endl;
//	}

	rotors_[0]->cmd = rotors_[0]->cmd * (0.25 / 0.253738);
	rotors_[1]->cmd = rotors_[1]->cmd * (0.25 / 0.246083);
	rotors_[2]->cmd = rotors_[2]->cmd * (0.25 / 0.255279);
	rotors_[3]->cmd = rotors_[3]->cmd * (0.25 / 0.244770);

//	for (unsigned i = 0; i < rotors_.size(); ++i) {
//		std::cout << "scaled control:" << rotors_[i]->cmd << std::endl;
//	}

	  for (unsigned i = 0; i < rotors_.size(); ++i)
	  {
	    double velTarget = rotors_[i]->multiplier * rotors_[i]->cmd / rotors_[i]->rotorVelocitySlowdownSim;
	    double vel = rotors_[i]->joint->GetVelocity(0);
	    double error = vel - velTarget;
	    double force = rotors_[i]->pid.Update(error, dt);

//	    gzdbg << "joint" << i << " cmd:" << rotors_[i]->cmd <<
//	    " velTarget:" <<  velTarget << " currentVel:" << vel << " error:" << error
//	    << " force:" << force << " dt:" << dt.Double() << "\n";
	    rotors_[i]->joint->SetForce(0, force);
	  }
}

    void IrisController::SetControl(const geometry_msgs::Twist::ConstPtr& control_twist) {
        boost::mutex::scoped_lock lock(this->update_mutex_);
        targetThrottle = control_twist->linear.z;
        targetPitch = control_twist->angular.y;
        targetRoll = control_twist->angular.x;
        targetYaw = control_twist->angular.z;
//        std::cout << "SetControl thorttle:" << targetThrottle << " pitch:" << targetPitch <<
//        		" roll:" << targetRoll << " yaw:" << targetYaw << std::endl;
    }

//    void IrisController::ProcessUserCommand(const std_msgs::String::ConstPtr& user_command) {
//    	gzdbg << "processing user command:" << user_command->data << "\n";
////    	planner.ProcessUserCommand(user_command->data);
//    }

    void IrisController::QueueThread() {
        static const double timeout = 0.01;
        while (this->rosnode_->ok()) {
            this->queue_.callAvailable(ros::WallDuration(timeout));
        }
    }
}
