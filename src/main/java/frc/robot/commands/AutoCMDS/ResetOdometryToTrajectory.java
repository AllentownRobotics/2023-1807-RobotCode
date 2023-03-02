// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCMDS;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Utils.Constants.AutoContsants;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetOdometryToTrajectory extends InstantCommand {
  PathPlannerTrajectory autoTrajectory = null;
  DriveTrain drive = null;
 
  public ResetOdometryToTrajectory(String path, DriveTrain drive) {
    autoTrajectory = PathPlanner.loadPath(path, AutoContsants.AUTO_MAX_SPEED_MPS, AutoContsants.AUTO_MAX_ACCELERATION_MPS_SQUARED);
    this.drive = drive;
   
    addRequirements(drive);
    
  }


  @Override
  public void initialize() {
    drive.resetOdometry(autoTrajectory.getInitialHolonomicPose());
  }
}
