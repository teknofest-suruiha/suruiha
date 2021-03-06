/*
 * util.cpp
 *
 *  Created on: May 2, 2018
 *      Author: okan
 */

#include <suruiha_gazebo_plugins/util/util.h>
#include <geometry_msgs/Pose.h>
#include <gazebo/common/common.hh>
#include <iostream>

namespace gazebo {

	bool Util::FindString(std::string haystack, std::string needle) {
		size_t found = haystack.find(needle);
		if (found != std::string::npos) {
			return true;
		} else {
			return false;
		}
	}

	bool Util::GetSdfParam(sdf::ElementPtr _sdf, const std::string &_name,
			double &_param, const double &_defaultValue, const bool &_verbose) {
		if (_sdf->HasElement(_name)) {
			_param = _sdf->GetElement(_name)->Get<double>();
			return true;
		}

		_param = _defaultValue;
		if (_verbose) {
			std::cerr << "Please specify a value for parameter [" << _name
					<< "].\n";
		}
		return false;
	}

    geometry_msgs::Pose Util::FromIgnitionPose(ignition::math::Pose3d pose) {
        geometry_msgs::Pose geomPose;
        geomPose.position.x = pose.Pos().X();
        geomPose.position.y = pose.Pos().Y();
        geomPose.position.z = pose.Pos().Z();
        geomPose.orientation.x = pose.Rot().X();
        geomPose.orientation.y = pose.Rot().Y();
        geomPose.orientation.z = pose.Rot().Z();
        geomPose.orientation.w = pose.Rot().W();
        return geomPose;
    }

    void Util::GetModels(std::map<std::string, physics::ModelPtr>& models, int maxCount,
                         const std::string baseModelName, physics::WorldPtr worldPtr) {
        int i = 0;
        for (i = 0; i < maxCount; i++) {
            std::stringstream ss;
            ss << baseModelName << i;
            gzdbg << "Checking model with name " << ss.str() << std::endl;
            physics::ModelPtr modelPtr = worldPtr->ModelByName(ss.str());
            if (modelPtr == NULL) {
                break;
            } else {
                models.insert(std::pair<std::string, physics::ModelPtr>(ss.str(), modelPtr));
            }
        }
        gzdbg << "We have " << i << " many " << baseModelName << " models" << std::endl;
    }


	void Util::Rotate(ignition::math::Vector2d& myVec, float radian) {
		ignition::math::Vector2d rotated;
		rotated.X(myVec.X() * cos(radian) - myVec.Y() * sin(radian));
		rotated.Y(myVec.X() * sin(radian) + myVec.Y() * cos(radian));
		myVec.X(rotated.X());
		myVec.Y(rotated.Y());
	}

    ros::NodeHandle* Util::CreateROSNodeHandle(std::string namespace_) {
        // Make sure the ROS node for Gazebo has already been initalized
        if (!ros::isInitialized()) {
            ROS_FATAL_STREAM_NAMED("ros_node_creation", "A ROS node for Gazebo has not been initialized, unable to load plugin.");
            return NULL;
        }

        ros::NodeHandle* node = new ros::NodeHandle(namespace_);
        return node;
    }

	double Util::CalDist(const ignition::math::Pose3d& modelPose, const geometry_msgs::Pose& trackingPose) {
		double dist = pow(modelPose.Pos().X() - trackingPose.position.x, 2) +
				pow(modelPose.Pos().Y() - trackingPose.position.y, 2);
		dist = sqrt(dist);
		return dist;
	}

	double Util::CalDist(const ignition::math::Pose3d& pose1, const ignition::math::Pose3d& pose2) {
		double dist = pow(pose1.Pos().X() - pose2.Pos().X(), 2) +
					  pow(pose1.Pos().Y() - pose2.Pos().Y(), 2) +
		              pow(pose1.Pos().Z() - pose2.Pos().Z(), 2);
		dist = sqrt(dist);
		return dist;
	}

	std::pair<double, double> Util::GetStartEndTimeOfActor(sdf::ElementPtr actorSDF) {
		sdf::ElementPtr scriptElement = actorSDF->GetElement("script");
		double startTime = 0;
		scriptElement->GetElement("delay_start")->GetValue()->Get(startTime);

		sdf::ElementPtr child = scriptElement->GetElement("trajectory")->GetFirstElement();
		double maxEndTime = 0;
		for (; child; child = child->GetNextElement()) {
			if (child->GetName() == "waypoint") {
				double time = 0.0;
				child->GetElement("time")->GetValue()->Get(time);
				if (time > maxEndTime) {
					maxEndTime = time;
				}
			}
		}
		return std::pair<double, double>(startTime, startTime+maxEndTime);
	}

	/**
	 * Custom string split function to get model and sensor name from scoped name
	 * @param scopedName
	 * @return
	 */
	std::pair<std::string, std::string> Util::GetModelAndSensorName(const std::string scopedName) {
		std::vector<std::string> tokens;
		unsigned int tokenStart = 0;
		for (unsigned int i = 0; i < scopedName.size()-1; i++) {
			if (scopedName[i] == ':' && scopedName[i+1] == ':') {
				tokens.push_back(scopedName.substr(tokenStart, i-tokenStart));
				tokenStart = i+2;
			}
		}
		tokens.push_back(scopedName.substr(tokenStart, scopedName.size()-tokenStart));

		// testing begin
//		std::cout << "--- print tokens" << std::endl;
//		for (unsigned int i = 0; i < tokens.size(); i++) {
//			std::cout << i << ":" << tokens[i] << std::endl;
//		}
		// testing end

		return std::pair<std::string, std::string>(tokens[1], tokens[tokens.size()-1]);
	}

//
//	ignition::math::Vector3d Util::toEuler(ignition::math::Quaterniond& quad) {
//		quad.Euler().Z()
//
//	}

}

