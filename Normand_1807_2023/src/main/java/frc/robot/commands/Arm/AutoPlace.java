// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Enums.ClawState;
import frc.robot.commands.Arm.LowLevelCommands.SetArmAngle;
import frc.robot.commands.Claw.LowLevelCommands.SetClawState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlace extends SequentialCommandGroup {
  /**
   * Sequential command group which moved the arm to the desired angle then places the currently held game piece.
   * NOTE: this commands does not reset the arm to its reset position
   * To achieve this call {@link frc.robot.commands.Arm.ResetArm} after this command
   * Ends 0.1 seconds (100 ms) after the game piece is placed
   * @param arm Arm subsystem
   * @param claw Claw subsystem
   * @param angle Angle for the arm to place at
   */
  public AutoPlace(Arm arm, Claw claw, double angle) {
    addCommands(new SetArmAngle(arm, angle), 
                Commands.waitUntil(arm::atSetPoint),
                new WaitCommand(0.25),
                new SetClawState(claw, ClawState.Open),
                new WaitCommand(0.1));
  }
}
