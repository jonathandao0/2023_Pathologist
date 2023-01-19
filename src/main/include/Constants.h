// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
namespace ArmConstants {
    const int bottomArmMotorID = 4;
    const int topArmMotorID = 0;
    const int intakeTiltMotorID = 0;

    const double kBottomP = 0.1;
    const double kBottomD = 0.1;

    const double kTopP = 0.0;
    const double kTopD = 0.0;

    const double intakeAngleConversion = 0.0; 
    const double radiansToEncoder = 0.17825353626;

    const double totalArmLength = 0.0;
    const double bottomJointLength = 10.0;
    const double topJointLength = 10.0;

    const double xOriginAdjustment = 0.0;
    const double yOriginAdjustment = 0.0;
}

namespace OperatorConstants {

constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants
