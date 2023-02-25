// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.High;
import frc.robot.commands.Arm.Low;
import frc.robot.commands.Arm.Mid;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class TiaAuto extends SequentialCommandGroup {
    public TiaAuto(DriveSubsystem driveSubsystem, Arm armSubsystem)
    {
      addCommands(
        new High(true, armSubsystem),
        new ResetOdometrytoTrajectory("tia", driveSubsystem),
        new ParallelDeadlineGroup(
          new FollowPath("tia", driveSubsystem).getCommand(),
          new Low(true, armSubsystem)),
        new ResetOdometrytoTrajectory("tia2", driveSubsystem),
        new FollowPath("tia2", driveSubsystem).getCommand());
    }
}
