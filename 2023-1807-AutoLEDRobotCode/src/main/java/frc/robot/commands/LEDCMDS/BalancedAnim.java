// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDCMDS;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LED;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalancedAnim extends SequentialCommandGroup {
  LED light;

  public BalancedAnim(LED light) {
    this.light = light;
    addCommands(Commands.runOnce(() -> light.setAllianceColor()),
    Commands.waitSeconds(0.5),
    Commands.runOnce(() -> light.setLEDs(0, 0, 0)),
    Commands.waitSeconds(0.5));
  }
}
