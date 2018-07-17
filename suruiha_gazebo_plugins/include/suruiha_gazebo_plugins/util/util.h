/*
 * util.h
 *
 *  Created on: May 2, 2018
 *      Author: okan
 */

#ifndef INCLUDE_SURUIHA_GAZEBO_PLUGINS_UTIL_H_
#define INCLUDE_SURUIHA_GAZEBO_PLUGINS_UTIL_H_

#include <string>
#include <sdf/sdf.hh>
#include <geometry_msgs/Pose.h>
#include <ignition/math/Pose3.hh>

namespace gazebo {
class Util {
	public: static bool FindString(std::string haystack, std::string needle);
	public: static bool GetSdfParam(sdf::ElementPtr _sdf, const std::string &_name,
	  double &_param, const double &_defaultValue, const bool &_verbose = false);

//	public: static void rotate(ignition::math::Vector2d& myVec, float radian);
//	public: static ignition::math::Vector3d toEuler(ignition::math::Quaterniond& quad);
    public: static geometry_msgs::Pose fromIgnitionPose(ignition::math::Pose3d& pose);
};
}



#endif /* INCLUDE_SURUIHA_GAZEBO_PLUGINS_UTIL_H_ */
