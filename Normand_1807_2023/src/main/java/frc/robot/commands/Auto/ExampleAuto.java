// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class ExampleAuto extends SequentialCommandGroup {
    public ExampleAuto(DriveSubsystem driveSubsystem, Arm armSubsystem)
    {
      addCommands(
        /*new High(true, armSubsystem),
        new ResetOdometrytoTrajectory("straight", driveSubsystem),
        new ParallelDeadlineGroup(
          new FollowPath("straight", driveSubsystem).getCommand(),
          new Mid(true, armSubsystem)),
        new ResetOdometrytoTrajectory("straightp2", driveSubsystem),
        new ParallelDeadlineGroup(
          new FollowPath("straightp2", driveSubsystem).getCommand(),
          new High(true, armSubsystem)),
          new Mid(true,armSubsystem)*/);
    }
}
