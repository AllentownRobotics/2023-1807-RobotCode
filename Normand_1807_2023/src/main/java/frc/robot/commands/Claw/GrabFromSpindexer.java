// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Enums.ClawState;
import frc.robot.Enums.WristState;
import frc.robot.commands.Claw.LowLevelCommands.SetClawState;
import frc.robot.commands.Claw.LowLevelCommands.SetWristState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabFromSpindexer extends SequentialCommandGroup {
  /**
   * Sequential command group which waits for the arm to be reset before running.
   * Grabs the game piece from the spindexer and puts the wrist into position to run the arm.
   * Ends after the wrist is set to the ready position
   * @param claw Claw subsystem
   * @param arm Arm subsystem
   */
  public GrabFromSpindexer(Claw claw, Arm arm) {
    addCommands(Commands.waitUntil(arm::atReset),
                new SetWristState(claw, WristState.WristDown),
                new WaitCommand(0.1),
                new SetClawState(claw, ClawState.Closed),
                new WaitCommand(0.2),
                new SetWristState(claw, WristState.WristOut));
  }
}
