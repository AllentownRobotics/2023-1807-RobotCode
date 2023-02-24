// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetArm extends SequentialCommandGroup {
  /** Creates a new ResetArm. */
  public ResetArm(RobotContainer rc) {
    RotateArmToSetPoint resetCheckPoint = new RotateArmToSetPoint(
      rc.arm, 35.0);
    WaitCommand timer = new WaitCommand(0.75);
    RotateArmToSetPoint finalReset = new RotateArmToSetPoint(rc.arm, 11.0);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(resetCheckPoint, timer, finalReset);
  }
}
