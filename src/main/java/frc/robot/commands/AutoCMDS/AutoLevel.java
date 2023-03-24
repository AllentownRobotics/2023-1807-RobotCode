// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCMDS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoLevel extends CommandBase {
   boolean shouldSlow = false; 

  private DriveTrain s_Swerve;
  PIDController kLevelingPID = new PIDController(0.045, 0.00, 0.00);

  /**
   * Command which controls the robot to automatically level on the charge station.
   * Runs until interrupted
   */
  public AutoLevel() {
      s_Swerve = DriveTrain.getInstance();
      addRequirements(s_Swerve);
      kLevelingPID.setIntegratorRange(-3, 3);
      //kLevelingPID.setTolerance(0.5);
  }

  @Override
  public void initialize(){
    shouldSlow = false;
    kLevelingPID.reset();
  }

  @Override
  public void execute() {
    if(Math.abs(s_Swerve.getRoll()) <= 2.5){
        shouldSlow = true;
    }
    double speed = -kLevelingPID.calculate(s_Swerve.getRoll(), 0);
    speed *= shouldSlow ? 0.5 : 1.0;
      s_Swerve.levelSet(speed);
  }

  @Override
  public boolean isFinished() {
      return false;
  }

}
