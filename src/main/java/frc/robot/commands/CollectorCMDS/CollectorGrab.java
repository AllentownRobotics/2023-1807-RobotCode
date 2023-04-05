// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CollectorCMDS;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Collector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CollectorGrab extends SequentialCommandGroup {
  Collector collector;
  /** Creates a new CollectorGrab. */
  public CollectorGrab() {
    collector = Collector.getInstance();
    addCommands(new CollectorIn(),
      new CollectorSpit(() -> -1.0).withTimeout(0.085));
  }
}
