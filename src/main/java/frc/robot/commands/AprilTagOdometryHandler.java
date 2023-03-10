// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AprilTagOdometryHandler extends ParallelCommandGroup {
  /** Creates a new AprilTagOdometryHandler. */
  public AprilTagOdometryHandler(DriveTrain drive, Limelight limelight) {
    addCommands(Commands.repeatingSequence(Commands.waitSeconds(0.75).andThen(
          Commands.runOnce(() -> drive.resetOdometry(limelight.robotPoseAllianceSpace())))),
      Commands.runOnce(() -> limelight.resetLocalOdometryPosition(drive.getHeading(), drive.getModulePositions())));
  }
}
