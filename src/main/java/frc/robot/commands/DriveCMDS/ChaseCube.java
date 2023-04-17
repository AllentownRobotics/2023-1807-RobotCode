// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCMDS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class ChaseCube extends CommandBase {
  private PIDController kturningPID = new PIDController(0.015, 0, 0.0001);

  private DriveTrain drive;

  private CommandXboxController driveController;
  private CommandXboxController opController;

  public ChaseCube(CommandXboxController driveController, CommandXboxController opController) {
    drive = DriveTrain.getInstance();

    this.driveController = driveController;
    this.opController = opController;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kturningPID.reset();
    kturningPID.setTolerance(1.0);
    kturningPID.setSetpoint(0.0);

    Limelight.getInstance().setPipeline(Limelight.purplePipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!Limelight.tv){
      drive.drive(
        MathUtil.applyDeadband(driveController.getLeftY(), 0.1),
        MathUtil.applyDeadband(driveController.getLeftX(), 0.1),
        MathUtil.applyDeadband(-driveController.getRightX(), 0.1),
        true, true);
    }
    else{
      driveController.getHID().setRumble(RumbleType.kBothRumble, 0.5);
      opController.getHID().setRumble(RumbleType.kBothRumble, 0.5);
      
      double omega = kturningPID.calculate(Limelight.x);
      double speed = kturningPID.atSetpoint() ? 1.0 : 0.5;
      
      drive.drive(speed, 0.0, omega, false, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    opController.getHID().setRumble(RumbleType.kBothRumble, 0.0);

    Limelight.getInstance().setPipeline(Limelight.April2DPipeline);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
