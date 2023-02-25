// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

public class TranslateToTag extends CommandBase {
  double dx;
  PIDController pidController;
  Vision visionSubsystem;
  DriveSubsystem driveSubsystem;
  public TranslateToTag(Vision visionSubsystem, DriveSubsystem driveSubsystem) {
    pidController = new PIDController(0.01,0,0);
    pidController.setSetpoint(0);
    pidController.setTolerance(1);
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    addRequirements(visionSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dx = visionSubsystem.getDistanceX();
    driveSubsystem.translateRobot(pidController.calculate(dx));
    SmartDashboard.putNumber("x", dx);
    SmartDashboard.putNumber("pid", pidController.calculate(dx));
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
