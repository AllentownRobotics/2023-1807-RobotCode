// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SpindexerCMDS;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Spindexer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetSpindexRotations extends InstantCommand {
 
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetSpindexerRotations extends InstantCommand {
  Spindexer spindexer;

  double rotations;

  /**
   * Instant command that sets the desired number of rotations for the spindexer to spin then instantly ends
   * NOTE: This command does not wait for the spindexer to be at the desired rotations before ending
   * To achieve this use a {@code Commands.waitUntil} command 
   * passing in the spindexer's {@code atSetPoint()} as a boolean supplier Ex. 
   * {@code Commands.waitUntil(spindexer::atSetPoint)}
   * @param spindexer
   * @param rotations
   */
  public SetSpindexerRotations(Spindexer spindexer, double rotations) {
    this.spindexer = spindexer;
    this.rotations = rotations;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    spindexer.setDesiredRotation(rotations);
  }
}
}
