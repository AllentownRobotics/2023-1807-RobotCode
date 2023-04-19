// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CollectorCMDS;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Utils.LightAnimation;
import frc.robot.Utils.Enums.ClawState;
import frc.robot.Utils.Enums.WristState;
import frc.robot.commands.LightCMDS.FlashColor;
import frc.robot.commands.LightCMDS.SetAnimation;
import frc.robot.subsystems.Collector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CollectorIn extends SequentialCommandGroup {
  Collector collector;

  private int[] signalOutput;
  /** Creates a new CollectorIn. */
  public CollectorIn() {
    collector = Collector.getInstance();
    addCommands(
      new SetAnimation(LightAnimation.nullAnim),
      Commands.runOnce(() -> {signalOutput = collector.pieceInRange() ? new int[]{0, 186, 0} : new int[]{186, 0, 0};}),
      Commands.runOnce(() -> collector.setGripState(ClawState.Closed)),
      Commands.waitSeconds(0.2),
      Commands.runOnce(() -> collector.setFlipState(WristState.WristIn)).alongWith(new FlashColor(() -> signalOutput, 1.0)));
  }
}
