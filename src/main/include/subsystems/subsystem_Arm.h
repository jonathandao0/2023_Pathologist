// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"

class subsystem_Arm : public frc2::SubsystemBase {
 public:
  subsystem_Arm();

  double CalculateBottomArmAngle(double x, double y);
  double CalculateTopArmAngle(double x, double y);

  void MoveArm(double x, double y);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  rev::CANSparkMax m_BottomArmMotor;
  rev::CANSparkMax m_TopArmMotor;

  rev::SparkMaxPIDController m_BottomArmPID;
  rev::SparkMaxPIDController m_TopArmPID;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
