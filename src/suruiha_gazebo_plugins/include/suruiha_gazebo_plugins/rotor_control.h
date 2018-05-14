/*
 * RotorControl.h
 *
 *  Created on: May 10, 2018
 *      Author: okan
 */

#ifndef INCLUDE_SURUIHA_GAZEBO_PLUGINS_ROTOR_CONTROL_H_
#define INCLUDE_SURUIHA_GAZEBO_PLUGINS_ROTOR_CONTROL_H_

#include <gazebo/common/PID.hh>
#include <gazebo/physics/Joint.hh>
#include <ignition/math.hh>
#include <string>

namespace gazebo {
/// \brief Rotor class
class RotorControl
{
  /// \brief Constructor
  public: RotorControl();

  /// \brief rotor id
  public: int id = 0;

  /// \brief Max rotor propeller RPM.
  public: double maxRpm = 838.0;

  /// \brief Next command to be applied to the propeller
  public: double cmd = 0;

  /// \brief Velocity PID for motor control
  public: common::PID pid;

  /// \brief Control propeller joint.
  public: std::string jointName;

  /// \brief Control propeller joint.
  public: physics::JointPtr joint;

  /// \brief direction multiplier for this rotor
  public: double multiplier = 1;

  /// \brief unused coefficients
  public: double rotorVelocitySlowdownSim;
  public: double frequencyCutoff;
  public: double samplingRate;
  public: ignition::math::OnePole<double> velocityFilter;
};

}



#endif /* INCLUDE_SURUIHA_GAZEBO_PLUGINS_ROTOR_CONTROL_H_ */
