// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "SwerveModule.h"
#include "Constants.h"
#include <vector>
#include <units/velocity.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/controller/PIDController.h>
#include <ctre/phoenix/sensors/WPI_Pigeon2.h>
#include <frc2/command/CommandBase.h>
#include <frc/geometry/Rotation2d.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/interfaces/Gyro.h>
#include <frc/Filesystem.h>
#include <frc/Timer.h>
#include <wpi/fs.h>
#include <cmath>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include "subsystems/subsystem_DriveTrain.h"
// #include <pathplanner/lib/PathPlanner.h>
#include <frc/Timer.h>
#include "Constants.h"



class subsystem_DriveTrain:
  public frc2::SubsystemBase {
  public:
    subsystem_DriveTrain();
    void SwerveDrive(units::meters_per_second_t xSpeed,
                    units::meters_per_second_t ySpeed,
                    units::radians_per_second_t zRot,
                    bool FieldRelative, 
                    bool IsOpenLoop);
    void SwerveDrive(frc::Translation2d Translation, 
                    units::degrees_per_second_t Rotation,
                    bool FieldRelative, 
                    bool IsOpenLoop  );
    void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);
    void SwapOrientation();
    double SetThrottle(double input);
    void ChangeThrottle();

    void ZeroGyro();
    void ResetOdometry(frc::Pose2d Pose);
    frc::Pose2d GetPose();
    frc::Rotation2d GetYaw();

    void SetStationBalance();
    units::meters_per_second_t CalculatePitch();
    units::meters_per_second_t CalculateRoll();

    void SetTrajectory();
    void SetTimerAndTrajectory();
    bool FeLiNa();
    void ChangeReset(bool newReset);

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

  private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
    
    ctre::phoenix::sensors::WPI_Pigeon2 m_Gyro;

    SwerveModule m_FrontLeftModule;
    SwerveModule m_FrontRightModule;
    SwerveModule m_BackLeftModule;
    SwerveModule m_BackRightModule;

    double DegreeOfThrottle;

    frc::SwerveDriveKinematics<4> m_kinematics{
        SwerveConstants::m_FrontLeft, SwerveConstants::m_FrontRight, SwerveConstants::m_BackLeft,
        SwerveConstants::m_BackRight};


    frc::SwerveDrivePoseEstimator<4> m_PoseEstimator;
    frc::PIDController m_PID;
    frc::Timer m_Timer{};
    frc::Trajectory m_Trajectory;

    bool StartBalance;
    bool m_ToReset;

    subsystem_DriveTrain* m_DriveTrain;
    

    
    //std::vector<SwerveModule> m_Mods;
                           
  
};
