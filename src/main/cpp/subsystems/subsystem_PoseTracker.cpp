// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_PoseTracker.h"



subsystem_PoseTracker::subsystem_PoseTracker() = default;


std::pair<frc::Pose2d, units::millisecond_t> subsystem_PoseTracker::getEstimatedGlobalPose(
    frc::Pose3d prevEstimatedRobotPose){
        estimator.SetReferencePose(prevEstimatedRobotPose);
  units::millisecond_t currentTime = frc::Timer::GetFPGATimestamp();
  auto result = estimator.Update();
  if (result.second) {
    return std::make_pair<>(result.first.ToPose2d(),
                            currentTime - result.second);
  } else {
    return std::make_pair(frc::Pose2d(), 0_ms);
  }
    }
// This method will be called once per scheduler run
void subsystem_PoseTracker::Periodic() {
  
    yaw = target.GetYaw();
    pitch = target.GetPitch();
    area = target.GetArea();
    targetID = target.GetFiducialId();
    
    cameras.push_back(std::make_pair(cameraOne, robotToCam));
}
