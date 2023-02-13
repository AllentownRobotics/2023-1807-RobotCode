// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveCmd extends CommandBase {
  double xSpeed = 0.0;
  double ySpeed = 0.0;
  double rot = 0.0;
  /** Creates a new DriveCmd. */
  public DriveCmd(double xSpeed, double ySpeed, double rot) {
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rot = rot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_robotDrive.drive(
      xSpeed,
      ySpeed,
      rot);
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
