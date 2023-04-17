// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCMDS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class StrafeTarget extends CommandBase {
  DriveTrain drive;

  CommandXboxController driveController;

  private PIDController kturningPID = new PIDController(0.015, 0, 0.0001);
  private PIDController kStrafingPID = new PIDController(0.018, 0.000, 0.000);

  public StrafeTarget(CommandXboxController controller) {
    drive = DriveTrain.getInstance();
    driveController = controller;

    kturningPID.enableContinuousInput(-180, 180);
    kturningPID.setTolerance(1);
    kStrafingPID.setTolerance(0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Limelight.tv){
      drive.drive(MathUtil.applyDeadband(driveController.getLeftY(), 0.3), kStrafingPID.calculate(MathUtil.applyDeadband(Limelight.x, 0.0)), kturningPID.calculate(drive.getHeadingDegrees(), 0), false, false);
    }
    else{
      drive.drive(MathUtil.applyDeadband(driveController.getLeftY(), 0.15), MathUtil.applyDeadband(driveController.getLeftX(), 0.15), kturningPID.calculate(drive.getHeadingDegrees(), 0), false, false);
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
