// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_DriveAuton.h"

command_DriveAuton::command_DriveAuton(subsystem_DriveTrain* DriveTrain, bool ToReset):
m_DriveTrain{DriveTrain}, m_ToReset{ToReset} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_DriveTrain});
  // fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  // deployDirectory = deployDirectory / "pathplanner" / TrajFilePath;
  // m_Trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
}



// Called when the command is initially scheduled.
void command_DriveAuton::Initialize() {
  m_DriveTrain->ChangeReset(m_ToReset);
  m_DriveTrain->SetTrajectory();
  m_DriveTrain->SetTimerAndTrajectory();
  
}

// Called repeatedly when this Command is scheduled to run
void command_DriveAuton::Execute() {}

// Called once the command ends or is interrupted.
void command_DriveAuton::End(bool interrupted) {
  m_DriveTrain -> SwerveDrive(0_mps, 0_mps, 0_rad_per_s, false, false);
}

// Returns true when the command should end.
bool command_DriveAuton::IsFinished() {
  return m_DriveTrain->FeLiNa();
}
