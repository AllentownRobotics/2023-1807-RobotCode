// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LightCMDS;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.Lights;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlinkColor extends RepeatCommand {
  /** Creates a new BlinkColor. */
  public BlinkColor(Supplier<int[]> colorSupplier, double timeOnColorSeconds, double timeOffColorSeconds) {
    super(new FlashColor(colorSupplier, timeOnColorSeconds).andThen(Commands.waitSeconds(timeOffColorSeconds)));
    addRequirements(Lights.getInstance());
  }
  public BlinkColor(int[] color, double timeOnColorSeconds, double timeOffColorSeconds){
    this(() -> color, timeOnColorSeconds, timeOffColorSeconds);
  }
}
