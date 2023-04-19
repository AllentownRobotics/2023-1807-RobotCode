// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CollectorCMDS;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Utils.LightAnimation;
import frc.robot.Utils.Enums.ClawState;
import frc.robot.Utils.Enums.WristState;
import frc.robot.commands.LightCMDS.BlinkColor;
import frc.robot.commands.LightCMDS.SetAnimation;
import frc.robot.subsystems.Collector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CollectorOut extends SequentialCommandGroup {
  Collector collector;
  /** Creates a new CollectorOut. */
  public CollectorOut() {
    collector = Collector.getInstance();
    addCommands(Commands.runOnce(() -> collector.setFlipState(WristState.WristOut)),
      Commands.runOnce(() -> collector.setGripState(ClawState.Open)),
      new SetAnimation(LightAnimation.nullAnim),
      new BlinkColor(new int[] {186, 0, 0}, 0.1, 0.1));
  }
}
