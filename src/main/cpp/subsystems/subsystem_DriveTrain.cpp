
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include "subsystems/subsystem_DriveTrain.h"
#include "frc/smartdashboard/SmartDashboard.h"



subsystem_DriveTrain::subsystem_DriveTrain():
    m_Gyro{SwerveConstants::CANCoderID}, 
    m_FrontLeftModule{FrontLeftModule::Constants},
    m_FrontRightModule{FrontRightModule::Constants},
    m_BackLeftModule{BackLeftModule::Constants},
    m_BackRightModule{BackRightModule::Constants},
    m_PoseEstimator{m_kinematics,
      frc::Rotation2d{},
      {m_FrontLeftModule.GetPosition(), m_FrontRightModule.GetPosition(),
       m_BackLeftModule.GetPosition(), m_BackRightModule.GetPosition()},
      frc::Pose2d{}}, 
    m_PID{SwerveConstants::BalancekP, SwerveConstants::BalancekI, SwerveConstants::BalancekD}
{
    m_Gyro.ConfigFactoryDefault();
    ZeroGyro();
    m_PID.SetSetpoint(0.0);
    m_PID.SetTolerance(5.0);
    DegreeOfThrottle = 1;
    StartBalance = false;
}

void subsystem_DriveTrain::SwerveDrive(units::meters_per_second_t xSpeed,
                                        units::meters_per_second_t ySpeed,
                                        units::radians_per_second_t zRot,
                                        bool FieldRelative, 
                                        bool IsOpenLoop){
    if(StartBalance){
        // xSpeed += CalculateRoll();
        // ySpeed += CalculatePitch();
    }
    auto moduleStates = SwerveConstants::m_kinematics.ToSwerveModuleStates(
        FieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                            xSpeed, ySpeed, zRot, GetYaw()
                        ): frc::ChassisSpeeds{xSpeed, ySpeed, zRot});
    SwerveConstants::m_kinematics.DesaturateWheelSpeeds(&moduleStates, SwerveConstants::MaxSpeed);
    auto [FrontLeft, FrontRight, BackLeft, BackRight] = moduleStates;
    //auto [FrontRight, RearRight,]

    m_FrontLeftModule.SetDesiredState(FrontLeft, IsOpenLoop);
    m_FrontRightModule.SetDesiredState(FrontRight, IsOpenLoop);
    m_BackLeftModule.SetDesiredState(BackLeft, IsOpenLoop);
    m_BackRightModule.SetDesiredState(BackRight, IsOpenLoop);

    frc::SmartDashboard::SmartDashboard::PutNumber("Front Left Angle", FrontLeft.angle.Degrees().value() );
    frc::SmartDashboard::SmartDashboard::PutNumber("Front Left Drive", FrontLeft.speed.value() );    

    frc::SmartDashboard::SmartDashboard::PutNumber("Front Right Angle", FrontRight.angle.Degrees().value() );
    frc::SmartDashboard::SmartDashboard::PutNumber("Front Right Drive", FrontRight.speed.value() );
    
    frc::SmartDashboard::SmartDashboard::PutNumber("Back Left Angle", BackLeft.angle.Degrees().value() );
    frc::SmartDashboard::SmartDashboard::PutNumber("Back Left Drive", BackLeft.speed.value() );

    frc::SmartDashboard::SmartDashboard::PutNumber("Back Right Angle", BackRight.angle.Degrees().value() );
    frc::SmartDashboard::SmartDashboard::PutNumber("Back Right Drive", BackRight.speed.value() );
}

void subsystem_DriveTrain::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates){
    SwerveConstants::m_kinematics.DesaturateWheelSpeeds(&desiredStates,
                                                AutoConstants::MaxSpeed);
    m_FrontLeftModule.SetDesiredState(desiredStates[0], false);
    m_FrontRightModule.SetDesiredState(desiredStates[1], false);
    m_BackLeftModule.SetDesiredState(desiredStates[2], false);
    m_BackRightModule.SetDesiredState(desiredStates[3], false);
}

void subsystem_DriveTrain::SwapOrientation(){
    m_FrontLeftModule.SwapOrientation();
}

double subsystem_DriveTrain::SetThrottle(double input){
    //Written by Ayush; everyone point and laugh at 
    //"Sussy baka" - Ayush Singhal 2023
    double shiftedThrottle = DegreeOfThrottle == 1 ? input : input < 0.0 ? -1 * pow(input, DegreeOfThrottle) : pow(input, DegreeOfThrottle);
    frc::SmartDashboard::SmartDashboard::PutNumber("TEST THROTTLE SHIFT", shiftedThrottle);
    
    frc::SmartDashboard::SmartDashboard::PutNumber("Input THROTTLE", input);
    frc::SmartDashboard::SmartDashboard::PutNumber("Throttle Degree", DegreeOfThrottle);

    if(DegreeOfThrottle == 1){
        return input;
    } else {
        return input < 0.0 ? -1 * pow(input, DegreeOfThrottle) : pow(input, DegreeOfThrottle);
    }    
    
    }

void subsystem_DriveTrain::ChangeThrottle(){
    if(DegreeOfThrottle == SwerveConstants::LinearThrottle){
        DegreeOfThrottle = SwerveConstants::NonLinearThrottle;
    } else {
        DegreeOfThrottle = SwerveConstants::LinearThrottle;
    }
}

void subsystem_DriveTrain::ResetOdometry(frc::Pose2d Pose){
    m_PoseEstimator.ResetPosition(m_Gyro.GetRotation2d(),
                            {m_FrontLeftModule.GetPosition(), 
                                m_FrontRightModule.GetPosition(),
                                m_BackLeftModule.GetPosition(),
                                m_BackRightModule.GetPosition()},
                            Pose);
}

frc::Pose2d subsystem_DriveTrain::GetPose(){
    return m_PoseEstimator.GetEstimatedPosition();
}

frc::Rotation2d subsystem_DriveTrain::GetYaw(){ 
    units::degree_t Yaw{ m_Gyro.GetYaw() };
    return (SwerveConstants::InvertGyro) ? frc::Rotation2d{360_deg - Yaw}: frc::Rotation2d{Yaw}; 
}

units::meters_per_second_t subsystem_DriveTrain::CalculatePitch(){
    
    return units::meters_per_second_t{m_PID.Calculate(m_Gyro.GetPitch())};
}

units::meters_per_second_t subsystem_DriveTrain::CalculateRoll(){
    return units::meters_per_second_t{m_PID.Calculate(m_Gyro.GetRoll())};
}

void subsystem_DriveTrain::SetStationBalance(){
    StartBalance = !StartBalance;
}

void subsystem_DriveTrain::ZeroGyro(){
    m_Gyro.SetYaw(0);
}

void subsystem_DriveTrain::SetTrajectory(){
    const frc::Pose2d sideStart{1.54_ft, 23.23_ft, frc::Rotation2d(180_deg)};
    const frc::Pose2d crossScale{23.7_ft, 6.8_ft, frc::Rotation2d(-160_deg)};

    std::vector<frc::Translation2d> interiorWaypoints{
        frc::Translation2d{14.54_ft, 23.23_ft},
        frc::Translation2d{21.04_ft, 18.23_ft}};

    frc::TrajectoryConfig config{12_fps, 12_fps_sq};
    config.SetReversed(true);
    m_Trajectory = frc::TrajectoryGenerator::GenerateTrajectory(sideStart, interiorWaypoints, crossScale, config);
}

void subsystem_DriveTrain::SetTimerAndTrajectory(){
    m_Timer.Stop();
    m_Timer.Reset();
    m_Timer.Start();

    if( m_ToReset ){
        m_DriveTrain->ResetOdometry(m_Trajectory.InitialPose());
    }
    frc::ProfiledPIDController<units::radians> ThetaController{10, 
                                                AutoConstants::AutoDriveKI, 
                                                AutoConstants::AutoDriveKD, 
                                                frc::TrapezoidProfile<units::radians>::Constraints{AutoConstants::MaxAngularSpeed,
                                                                                                 AutoConstants::MaxAngularAccel}};

    ThetaController.EnableContinuousInput(-units::radian_t{3.14}, units::radian_t{ 3.14} );

    frc2::SwerveControllerCommand<4> swerveControllerCommand(m_Trajectory,
                                                            [this]{return m_DriveTrain->GetPose();}, 
                                                            SwerveConstants::m_kinematics,
                                                            frc2::PIDController{10,10,10},
                                                            frc2::PIDController{10,10,10},
                                                            ThetaController,
                                                            [this](auto moduleStates) { m_DriveTrain->SetModuleStates(moduleStates); },
                                                            {m_DriveTrain});
}

bool subsystem_DriveTrain::FeLiNa(){
    return m_Timer.Get() >= (m_Trajectory.TotalTime() * 1.2);
}

void subsystem_DriveTrain::ChangeReset(bool newReset){
    m_ToReset = newReset;
}

// This method will be called once per scheduler run
void subsystem_DriveTrain::Periodic() {
    m_PoseEstimator.Update(m_Gyro.GetRotation2d(),
                        {m_FrontLeftModule.GetPosition(), 
                            m_FrontRightModule.GetPosition(),
                            m_BackLeftModule.GetPosition(),
                            m_BackRightModule.GetPosition()} );
}
