// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCMDS.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Utils.Enums.ClawState;
import frc.robot.commands.ArmCMDS.AutoPlace;
import frc.robot.commands.ArmCMDS.ResetArm;
import frc.robot.commands.AutoCMDS.AutoLevel;
import frc.robot.commands.AutoCMDS.FollowPath;
import frc.robot.commands.AutoCMDS.ResetOdometryToTrajectory;
import frc.robot.commands.ClawCMDS.LowLevelCMDS.SetClawState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeHighLeaveEngage extends SequentialCommandGroup {
  public ConeHighLeaveEngage(DriveTrain drive, Arm arm, Claw claw, RobotContainer rc)
  {
    addCommands(
      new SetClawState(claw, ClawState.Closed),
      new AutoPlace(arm, claw, 180.182),
      new ResetOdometryToTrajectory("ConeHighLeaveEngage", drive),
      new ParallelDeadlineGroup(
        new FollowPath("ConeHighLeaveEngage", 2, 2, drive).getCommand(),
        new ResetArm(rc)),
      new AutoLevel(drive),
      Commands.runOnce(() -> drive.setX(), drive));
  }
}