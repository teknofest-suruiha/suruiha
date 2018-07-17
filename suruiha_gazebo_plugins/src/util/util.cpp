/*
 * util.cpp
 *
 *  Created on: May 2, 2018
 *      Author: okan
 */

#include <suruiha_gazebo_plugins/util/util.h>
#include <geometry_msgs/Pose.h>

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

    geometry_msgs::Pose Util::fromIgnitionPose(ignition::math::Pose3d& pose) {
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

//
//	void Util::rotate(ignition::math::Vector2d& myVec, float radian) {
//		ignition::math::Vector2d rotated;
//		rotated.X(myVec.X() * cos(radian) - myVec.Y() * sin(radian));
//		rotated.Y(myVec.X() * sin(radian) + myVec.Y() * cos(radian));
//		myVec.X(rotated.X());
//		myVec.Y(rotated.Y());
//	}
//
//	ignition::math::Vector3d Util::toEuler(ignition::math::Quaterniond& quad) {
//		quad.Euler().Z()
//
//	}

}

