// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCMDS.LowLevelCMDS;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ManualArm extends CommandBase {
  Arm arm;
  DoubleSupplier axis;
  public ManualArm(Arm arm, DoubleSupplier axis) {
    //addRequirements(arm);
    withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    this.arm = arm;
    this.axis = axis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setAutomaticMode(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double correctedInput = Math.signum(axis.getAsDouble()) * Math.pow(MathUtil.applyDeadband(axis.getAsDouble(), 0.1), 2);
    double angularVelocity = correctedInput * ArmConstants.MANUAL_SPEED_MAX_DEGREESPERSECOND;
    arm.setManualSpeed(angularVelocity * (ArmConstants.MANUAL_INVERT ? -1.0 : 1.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setAutomaticMode(true);
    arm.setManualSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
