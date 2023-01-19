// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_Arm.h"

subsystem_Arm::subsystem_Arm() : 
m_BottomArmMotor{ArmConstants::bottomArmMotorID, rev::CANSparkMax::MotorType::kBrushless},
m_TopArmMotor{ArmConstants::topArmMotorID, rev::CANSparkMax::MotorType::kBrushless},
m_IntakeTiltMotor{ArmConstants::intakeTiltMotorID, rev::CANSparkMax::MotorType::kBrushless},
m_BottomArmPID{m_BottomArmMotor.GetPIDController()},
m_TopArmPID{m_TopArmMotor.GetPIDController()},
m_IntakeTiltPID{m_IntakeTiltMotor.GetPIDController()}
{
    m_BottomArmPID.SetP(ArmConstants::kBottomP);
    m_BottomArmPID.SetD(ArmConstants::kBottomD);
    m_TopArmPID.SetP(ArmConstants::kTopP);
    m_TopArmPID.SetD(ArmConstants::kTopD);
}

double subsystem_Arm::CalculateBottomArmAngle(double x, double y){
    topAngle = CalculateTopArmAngle(x, y);
    bottomAngle = atan(y / x) - atan((ArmConstants::topJointLength * sin(topAngle)) 
    / (ArmConstants::bottomJointLength + (ArmConstants::topJointLength * cos(topAngle))));
    return bottomAngle;
}
double subsystem_Arm::CalculateTopArmAngle(double x, double y){
    topAngle = -acos((pow(x,2) + pow(y,2) - pow(ArmConstants::bottomJointLength,2) - pow(ArmConstants::topJointLength,2))
    /(2*ArmConstants::topJointLength*ArmConstants::bottomJointLength));
    return topAngle;
}

void subsystem_Arm::MoveArm(double x, double y){
    double adjustedX = x + ArmConstants::xOriginAdjustment;
    double adjustedY = y + ArmConstants::yOriginAdjustment; 
    if(/*adjustedX <= ArmConstants::totalArmLength && adjustedY <= ArmConstants::totalArmLength*/ true){
        double convertedBottom = CalculateBottomArmAngle(adjustedX, adjustedY) * ArmConstants::radiansToEncoder;
        double convertedTop = CalculateTopArmAngle(adjustedX, adjustedY) * ArmConstants::radiansToEncoder;

        m_BottomArmPID.SetReference(convertedBottom, rev::ControlType::kPosition, 0);
        m_TopArmPID.SetReference(convertedTop, rev::ControlType::kPosition, 0);
    } else {
        printf("INVALID X OR Y INPUT");
    }
}

void subsystem_Arm::SetIntakeAngle(double angle)
{
    double adjustedAngle = angle - intakeAngleOffset;
    m_IntakeTiltPID.SetReference(adjustedAngle, rev::ControlType::kPosition, 0); 
}

void subsystem_Arm::Periodic() {
    intakeAngleOffset = bottomAngle * ArmConstants::intakeAngleConversion; /* - some constant bleh */
}
