// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CollectorCMDS;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Spindexer;

public class CollectorSpit extends CommandBase {
  Collector collector;
  Spindexer spindexer;
  DoubleSupplier multiplier;
  public CollectorSpit(DoubleSupplier multiplier) {
    collector = Collector.getInstance();
    spindexer = Spindexer.getInstance();
    this.multiplier = multiplier;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    collector.spit(multiplier.getAsDouble());
    spindexer.runRoller(-multiplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted){
    collector.spit(0.0);
    spindexer.runRoller(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
