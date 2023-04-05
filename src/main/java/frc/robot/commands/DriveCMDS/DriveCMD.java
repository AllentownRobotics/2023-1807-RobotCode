// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCMDS;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;

public class DriveCMD extends CommandBase {
 
  private BooleanSupplier fieldOriented;
    
  private DriveTrain drive;

  private CommandXboxController drivecontroller;

  public DriveCMD(CommandXboxController controller, BooleanSupplier fieldOriented) {
      drive = DriveTrain.getInstance();
      addRequirements(drive);

      this.fieldOriented = fieldOriented;
      this.drivecontroller = controller;
  }

  @Override
  public void execute() {
      drive.drive(
          MathUtil.applyDeadband(drivecontroller.getLeftY(), 0.1),
          MathUtil.applyDeadband(drivecontroller.getLeftX(), 0.1),
          MathUtil.applyDeadband(-drivecontroller.getRightX(), 0.1),
          fieldOriented.getAsBoolean(), true);
  }
}
