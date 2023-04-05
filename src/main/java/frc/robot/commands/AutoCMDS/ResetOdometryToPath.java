// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCMDS;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetOdometryToPath extends InstantCommand {
  PathPlannerTrajectory path;
  public ResetOdometryToPath(PathPlannerTrajectory path) {
    this.path = path;
  }

  @Override
  public void initialize() {
    DriveTrain.getInstance().resetOdometry(path.getInitialHolonomicPose());
  }
}
