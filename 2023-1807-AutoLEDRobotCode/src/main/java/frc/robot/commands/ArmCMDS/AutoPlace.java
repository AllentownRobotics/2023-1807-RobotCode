// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCMDS;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Utils.Enums.ClawState;
import frc.robot.Utils.Enums.WristState;
import frc.robot.commands.ArmCMDS.LowLevelCMDS.SetArmAngle;
import frc.robot.commands.ClawCMDS.LowLevelCMDS.SetClawState;
import frc.robot.commands.ClawCMDS.LowLevelCMDS.SetWristState;
import frc.robot.commands.LEDCMDS.ScoringAnim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.LED;

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
  public AutoPlace(Arm arm, Claw claw, double angle, LED light) {
    addCommands(new SetArmAngle(arm, angle), 
                new ParallelRaceGroup(Commands.waitUntil(arm::atSetPoint), new WaitCommand(3.25)),
                new WaitCommand(0.25),
                new SetClawState(claw, ClawState.Open),
                new WaitCommand(0.1),
                Commands.runOnce(() -> light.TranslateReqAndTransport()), 
                Commands.runOnce(() -> new ScoringAnim(light)));
  }
}