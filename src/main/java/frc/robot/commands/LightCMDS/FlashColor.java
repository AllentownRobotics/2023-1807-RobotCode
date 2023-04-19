// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LightCMDS;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Utils.LightAnimation;
import frc.robot.subsystems.Lights;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FlashColor extends SequentialCommandGroup {
  Lights lights;
  Supplier<int[]> colorSupplier;
  /** Creates a new FlashColor. */
  public FlashColor(int[] color, double timeOnColorSeconds) {
    this(() -> color, timeOnColorSeconds);
  }

  public FlashColor(Supplier<int[]> colorSupplier, double timeOnColorSeconds){
    lights = Lights.getInstance();
    this.colorSupplier = colorSupplier;
    addRequirements(lights);
    addCommands(
      new SetAnimation(LightAnimation.nullAnim),
      Commands.runOnce(() -> lights.setLEDs(colorSupplier.get()[0], colorSupplier.get()[1], colorSupplier.get()[2])),
      Commands.waitSeconds(timeOnColorSeconds),
      Commands.runOnce(() -> lights.setLEDs(0, 0, 0))
    );
  }
}
