// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCMDS;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Utils.Enums.ClawState;
import frc.robot.Utils.Enums.CycleState;
import frc.robot.commands.ArmCMDS.LowLevelCMDS.SetArmAngle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Lights;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WaitForPlace extends ParallelDeadlineGroup {
  /**
   * Parallel deadline group which rotates the arm to its placement position and waits for the operator to place
   * the currently held game piece.
   * Ends once the operator places the piece
   */
  public WaitForPlace(Arm arm, SetArmAngle angles) {
    super(Commands.waitUntil(() -> Claw.getInstance().getClawState().equals(ClawState.Open)));
    addCommands(Commands.runOnce(() -> Lights.getInstance().transitionToNewCycleState(CycleState.Scoring)).andThen(
      Commands.repeatingSequence(angles).withInterruptBehavior(InterruptionBehavior.kCancelSelf)));
  }
}
