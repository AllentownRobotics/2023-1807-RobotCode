// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClawCMDS;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Utils.Enums.ClawState;
import frc.robot.Utils.Enums.WristState;
import frc.robot.commands.ClawCMDS.LowLevelCMDS.SetClawState;
import frc.robot.commands.ClawCMDS.LowLevelCMDS.SetWristState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WristToStandBy extends ParallelCommandGroup {
 /**
   * Parallel command group which runs 2 instant commands in parallel and ends instantly
   * Sets the claw to its idle state with the claw open and the wrist out
   */
  public WristToStandBy() {
    addCommands(new SetClawState(ClawState.Open),
                new SetWristState(WristState.WristOut));
  }
}
