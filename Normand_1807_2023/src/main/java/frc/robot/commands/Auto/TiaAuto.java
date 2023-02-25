// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.ResetArm;
import frc.robot.commands.Arm.LowLevelCommands.SetArmAngle;
import frc.robot.commands.Claw.ToggleClaw;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class TiaAuto extends SequentialCommandGroup {
    public TiaAuto(DriveSubsystem driveSubsystem, Arm armSubsystem)
    {
      addCommands(
        new SetArmAngle(armSubsystem, 180),
        new ResetOdometrytoTrajectory("tia", driveSubsystem),
        new ParallelDeadlineGroup(
          new FollowPath("tia", driveSubsystem).getCommand(),
          new ResetArm(armSubsystem)),
        new ResetOdometrytoTrajectory("tia2", driveSubsystem),
        new FollowPath("tia2", driveSubsystem).getCommand(),
        new AutoLevel(driveSubsystem));
    }
}
