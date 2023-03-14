// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SpindexerCMDS;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spindexer;

public class RunAtSpeed extends CommandBase {
  Spindexer spindexer;
  
  DoubleSupplier inputSupplier;

  public RunAtSpeed(Spindexer spindexer, DoubleSupplier inputSupplier) {
    addRequirements(spindexer);

    this.spindexer = spindexer;
    this.inputSupplier = inputSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    spindexer.spindex(MathUtil.applyDeadband(inputSupplier.getAsDouble(), 0.08));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spindexer.spindex(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
