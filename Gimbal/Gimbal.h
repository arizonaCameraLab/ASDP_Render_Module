/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file Gimbal.h
  * @brief Classes to control orientation gimbals.
  *
 * @author ReliaSolve.
 * @date April 8th, 2025.
 */

#pragma once

#include <memory>
#include <string>

/// @brief Base class defining the interface for a gimbal control system.
class Gimbal
{
public:
  virtual ~Gimbal() = default;

  /// @brief Get the status of the gimbal.
  /// @return true if the gimbal is connected and operational, false otherwise.
  virtual bool Status() = 0;

  /// @brief Move to home position.
  /// @details The home position is defined as the position where the gimbal is
  /// on its limit switches. This routine waits until it arrives before returning.
  /// @throws std::runtime_error if the gimbal cannot be homed.
  virtual void Home() = 0;

  /// @brief Move to a specified absolute orientation at the speeds defined at construction.
  /// @details This routine waits until the gimbal arrives at the specified orientation before returning.
  /// @param yawDegrees Yaw angle in degrees.
  /// @param pitchDegrees Pitch angle in degrees.
  /// @throws std::runtime_error if the gimbal cannot be homed.
  virtual void MoveAbsolute(double yawDegrees, double pitchDegrees) = 0;

protected:
  // Hide the default constructor to prevent direct instantiation of the base class.
  Gimbal() = default;
};

// A fake Gimbal class that prints where it is going to and moves instantly.
class GimbalFake : public Gimbal
{
public:
  GimbalFake() : Gimbal() {}
  bool Status() override { return true; }
  void Home() override;
  void MoveAbsolute(double yawDegrees, double pitchDegrees) override;
};

class GimbalZaber_X_G_RST : public Gimbal
{
public:
  /// @brief Constructor for the GimbalZaber_X_G_RST class, using two devices each with one axis.
  /// @param comPortName The name of the serial port to use for communication.
  /// @param deviceID The device ID to use, default is 1.
  /// @param maxVelocityDeg The maximum velocity in degrees/second, default -1 leaves unchanged.
  /// @param maxAccelDeg The maximum acceleration in degrees/second^2, default -1 leaves unchanged.
  /// @throws std::runtime_error if the gimbal cannot be opened or initialized.
  GimbalZaber_X_G_RST(std::string comPortName, int deviceID = 1, double maxVelocityDeg = -1, double maxAccelDeg = -1);

  /// @brief Destructor for the GimbalZaber_X_G_RST class.
  ~GimbalZaber_X_G_RST();

  bool Status() override;
  void Home() override;
  void MoveAbsolute(double yawDegrees, double pitchDegrees) override;

protected:
  class GimbalZaber_X_G_RST_Impl;
  std::shared_ptr<GimbalZaber_X_G_RST_Impl> m_impl;
};

/// @brief Base class for various iOptron-based gimbals. The CEM40 and CEM70 are supported.
class Gimbal_iOptron : public Gimbal
{
public:

  /// @brief Destructor.
  ~Gimbal_iOptron();

  bool Status() override;
  void Home() override;
  void MoveAbsolute(double yawDegrees, double pitchDegrees) override;

protected:
  /// @brief Constructor for the Gimbal_iOptron class.
  /// @param comPortName The name of the serial port to use for communication.
  /// @param mountInfoReponse The expected response from the MountInfo command, specific
  /// to each type of device.
  /// @throws std::runtime_error if the gimbal cannot be opened or initialized.
  Gimbal_iOptron(std::string comPortName, std::string mountInfoResponse);

  class Gimbal_iOptron_Impl;
  std::shared_ptr<Gimbal_iOptron_Impl> m_impl;

  /// Move to a specified absolute orientation at the speeds defined at construction, without
  /// checking the last commanded position to avoid long moves.  This is used internally.
  void MoveAbsoluteRaw(double yawAdjusted, double pitchAdjusted, std::string hemisphere, bool fixBadSlew);
  double m_lastYawDegrees = 0;   ///< The last commanded yaw angle in degrees, used to avoid long moves.
  double m_lastPitchDegrees = 0; ///< The last commanded pitch angle in degrees, used to avoid long moves.
};

class Gimbal_iOptron_CEM40 : public Gimbal_iOptron
{
public:
  /// @brief Constructor for the Gimbal_iOptron_CEM40 class.
  /// @param comPortName The name of the serial port to use for communication.
  /// @throws std::runtime_error if the gimbal cannot be opened or initialized.
  Gimbal_iOptron_CEM40(std::string comPortName);
};

class Gimbal_iOptron_CEM70 : public Gimbal_iOptron
{
public:
  /// @brief Constructor for the Gimbal_iOptron_CEM70 class.
  /// @param comPortName The name of the serial port to use for communication.
  /// @throws std::runtime_error if the gimbal cannot be opened or initialized.
  Gimbal_iOptron_CEM70(std::string comPortName);
};
