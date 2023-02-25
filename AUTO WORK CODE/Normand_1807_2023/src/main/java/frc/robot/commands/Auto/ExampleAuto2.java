// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.High;
import frc.robot.commands.Arm.Mid;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class ExampleAuto2 extends SequentialCommandGroup {
    public ExampleAuto2(DriveSubsystem driveSubsystem)
    {
      addCommands(
        new ResetOdometrytoTrajectory("pathplanner/generatedJSON/funk.wpilib.json", driveSubsystem),
        new FollowPath("pathplanner/generatedJSON/funk.wpilib.json", driveSubsystem).getCommand().andThen(() -> RobotContainer.m_robotDrive.drive(0, 0, 0, false)));
    }
}
