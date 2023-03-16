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
public class ScoringAnim extends SequentialCommandGroup {
  LED light;
  /** Creates a new ScoringAnims. */
  public ScoringAnim(LED light) {
    this.light = light;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(Commands.runOnce(() -> light.ScoringAnimInit()), 
    Commands.waitSeconds(.3),
    Commands.runOnce(() -> light.setLEDs(light.GAMEPIECE_R, light.GAMEPIECE_G, light.GAMEPIECE_B)),
    Commands.waitSeconds(.1),
    Commands.runOnce(() -> light.setLEDs(0, 0, 0)),
    Commands.waitSeconds(.1),
    Commands.runOnce(() -> light.setLEDs(light.GAMEPIECE_R, light.GAMEPIECE_G, light.GAMEPIECE_B)));
  }
}
