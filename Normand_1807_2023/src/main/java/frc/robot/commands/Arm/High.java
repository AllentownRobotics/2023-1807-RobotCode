// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class High extends CommandBase {
  boolean cubeMode;
  public High(boolean mode) {
    cubeMode = mode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(cubeMode)
    {
      RobotContainer.m_Arm.set(ArmConstants.cubeHighAngle);
    }
    else
    {
      RobotContainer.m_Arm.set(ArmConstants.coneHighAngle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
