// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.Arm;

public class RotateArmToSetPoint extends InstantCommand {
  CommandXboxController controller;
  Arm arm;

  double coneSetPoint;
  double cubeSetPoint;

  public RotateArmToSetPoint(Arm arm, CommandXboxController controller, double coneAngle, double cubeAngle) {
    addRequirements(arm);

    this.arm = arm;
    this.controller = controller;
    coneSetPoint = coneAngle;
    cubeSetPoint = cubeAngle;
  }
  public RotateArmToSetPoint(Arm arm, double uniAngle) {
    addRequirements(arm);

    this.arm = arm;
    coneSetPoint = uniAngle;
    cubeSetPoint = uniAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean placeCube;
    try{
      placeCube = controller.leftBumper().getAsBoolean();
    }
    finally{
      placeCube = false;
    }

    arm.setDesiredAngle(placeCube ? cubeSetPoint : coneSetPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double error = Math.abs(arm.getAngle() - arm.getDesiredAngle());
    if (error <= 2.5){
      return true;
    }
    return false;
  }
}
