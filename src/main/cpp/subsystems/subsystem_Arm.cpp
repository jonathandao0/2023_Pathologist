// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_Arm.h"

subsystem_Arm::subsystem_Arm() : 
m_BottomArmMotor{ArmConstants::bottomArmMotorID, rev::CANSparkMax::MotorType::kBrushless},
m_TopArmMotor{ArmConstants::topArmMotorID, rev::CANSparkMax::MotorType::kBrushless},
m_BottomArmPID{m_BottomArmMotor.GetPIDController()},
m_TopArmPID{m_TopArmMotor.GetPIDController()}
{
    m_BottomArmPID.SetP(ArmConstants::kBottomP);
    m_BottomArmPID.SetD(ArmConstants::kBottomD);
    m_TopArmPID.SetP(ArmConstants::kTopP);
    m_TopArmPID.SetD(ArmConstants::kTopD);
}

double subsystem_Arm::CalculateBottomArmAngle(double x, double y){
    double topAngle = CalculateTopArmAngle(x, y);
    return atan(y / x) - atan((ArmConstants::topJointLength * sin(topAngle)) 
    / (ArmConstants::bottomJointLength + (ArmConstants::topJointLength * cos(topAngle))));
}
double subsystem_Arm::CalculateTopArmAngle(double x, double y){
    return -acos((pow(x,2) + pow(y,2) - pow(ArmConstants::bottomJointLength,2) - pow(ArmConstants::topJointLength,2))
    /(2*ArmConstants::topJointLength*ArmConstants::bottomJointLength));
}

void subsystem_Arm::MoveArm(double x, double y){
    double adjustedX = x + ArmConstants::xOriginAdjustment;
    double adjustedY = y + ArmConstants::yOriginAdjustment; 
    if(adjustedX <= ArmConstants::totalArmLength && adjustedY <= ArmConstants::totalArmLength){
        double convertedBottom = CalculateBottomArmAngle(adjustedX, adjustedY);
        double convertedTop = CalculateTopArmAngle(adjustedX, adjustedY);

        m_BottomArmPID.SetReference(CalculateBottomArmAngle(adjustedX, adjustedY), rev::ControlType::kPosition, 0);
        m_TopArmPID.SetReference(CalculateTopArmAngle(adjustedX, adjustedY), rev::ControlType::kPosition, 0);
    } else {
        printf("INVALID X OR Y INPUT");
    }
}


void subsystem_Arm::Periodic() {}
