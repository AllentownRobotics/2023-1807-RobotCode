// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SpindexerCMDS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Spindexer;

public class RunAtSpeed extends CommandBase {
  Spindexer spindexer;

  double directionMod;
  
  CommandXboxController controller;
  public RunAtSpeed(Spindexer spindexer, double directionMod, CommandXboxController controller) {
    addRequirements(spindexer);

    this.spindexer = spindexer;
    this.directionMod = directionMod;
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double varSpeed = directionMod == -1.0 ? controller.getLeftTriggerAxis() : controller.getRightTriggerAxis();

    spindexer.spindex(directionMod * varSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spindexer.spindex(0.0);
    spindexer.zeroEncoder();
    spindexer.setDesiredRotation(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
