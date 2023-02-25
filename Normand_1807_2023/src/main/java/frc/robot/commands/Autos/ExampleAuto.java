// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.High;
import frc.robot.commands.Arm.Mid;

/** Add your docs here. */
public class ExampleAuto extends SequentialCommandGroup {
    public ExampleAuto()
    {
      addCommands(
        /*new High(true),
        new ParallelDeadlineGroup(
          new FollowPath("straight", 2, 2),
          new Mid(true)),
        new ParallelDeadlineGroup(
          new FollowPath("straight2"),
          new High(true))*/
        new FollowPath2("pathplanner/generatedJSON/straight.wpilib.json").getCommand().andThen(() -> RobotContainer.m_robotDrive.drive(0, 0, 0, false)));
    }
}
