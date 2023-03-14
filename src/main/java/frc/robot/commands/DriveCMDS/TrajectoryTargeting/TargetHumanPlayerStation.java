// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCMDS.TrajectoryTargeting;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Utils.Constants.DriveConstants;
import frc.robot.commands.AutoCMDS.FollowPath;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TargetHumanPlayerStation extends SequentialCommandGroup {
  /** Creates a new TargetHumanPlayerStation. */
  public TargetHumanPlayerStation(DriveTrain drivetrain, Limelight limelight) {
    /*SwerveModuleState[] moduleStates = drivetrain.getModuleStates();
    ChassisSpeeds speeds = DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(moduleStates);
    addCommands(new FollowPath(limelight.generateSubstationTrajectory(
          drivetrain.getPose(), speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
        3.5, 3.0, drivetrain, drivetrain.getOdometryInstance()).getCommand());*/
  }
}
