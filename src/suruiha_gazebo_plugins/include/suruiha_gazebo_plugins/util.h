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

namespace gazebo {
class Util {
	public: static bool FindString(std::string haystack, std::string needle);
	public: static bool GetSdfParam(sdf::ElementPtr _sdf, const std::string &_name,
	  double &_param, const double &_defaultValue, const bool &_verbose = false);
};
}



#endif /* INCLUDE_SURUIHA_GAZEBO_PLUGINS_UTIL_H_ */
