// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_MoveArm.h"

command_MoveArm::command_MoveArm(subsystem_Arm *arm) : m_arm{arm} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_arm});
}

// Called when the command is initially scheduled.
void command_MoveArm::Initialize() {
  m_arm->MoveArm(0,20);
}

// Called repeatedly when this Command is scheduled to run
void command_MoveArm::Execute() {}

// Called once the command ends or is interrupted.
void command_MoveArm::End(bool interrupted) {}

// Returns true when the command should end.
bool command_MoveArm::IsFinished() {
  return false;
}
