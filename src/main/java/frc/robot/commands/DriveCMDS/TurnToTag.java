// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCMDS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class TurnToTag extends CommandBase {
  /** Creates a new TurnToTag. */
  double x;
  PIDController pidController;
  Vision vision;
  DriveTrain drive;
  public TurnToTag(Vision vision, DriveTrain drive) {
    pidController = new PIDController(0.05,0,0);
    pidController.setSetpoint(0);
    pidController.setTolerance(1);
    this.vision = vision;
    this.drive = drive;
    addRequirements(vision);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = vision.getVisionX();
    drive.turnRobot(pidController.calculate(x));
    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("pid", pidController.calculate(x));
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
