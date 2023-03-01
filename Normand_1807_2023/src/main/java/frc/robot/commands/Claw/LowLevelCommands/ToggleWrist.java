// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw.LowLevelCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;

public class ToggleWrist extends InstantCommand {
  Claw claw;

  /**
   * Instant command that toggles the wrist state and then instantly ends
   * @param claw Claw subsystem
   */
  public ToggleWrist(Claw claw) {
    this.claw = claw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.toggleWristState();
  }
}
