// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Enums.ClawState;
import frc.robot.commands.Arm.*;
import frc.robot.commands.Auto.AutoLevel;
import frc.robot.commands.Auto.FollowPath;
import frc.robot.commands.Auto.ResetOdometrytoTrajectory;
import frc.robot.commands.Claw.LowLevelCommands.SetClawState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class ConeHighLeaveEngageLeft extends SequentialCommandGroup {
    public ConeHighLeaveEngageLeft(DriveSubsystem driveSubsystem, Arm armSubsystem, Claw clawSubsystem, RobotContainer robotContainer)
    {
      addCommands(
        new SetClawState(clawSubsystem, ClawState.Closed),
        new AutoPlace(armSubsystem, clawSubsystem, 180.182),
        new ResetOdometrytoTrajectory("ConeHighLeaveEngageLeft", driveSubsystem),
        new ParallelDeadlineGroup(
          new FollowPath("ConeHighLeaveEngageLeft", 3, 3, driveSubsystem).getCommand(),
          new ResetArm(robotContainer)),
        new AutoLevel(driveSubsystem),
        Commands.runOnce(() -> driveSubsystem.setX(), driveSubsystem));
    }
}
