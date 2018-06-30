/*
 * util.cpp
 *
 *  Created on: May 2, 2018
 *      Author: okan
 */

#include <suruiha_gazebo_plugins/util.h>

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

}

