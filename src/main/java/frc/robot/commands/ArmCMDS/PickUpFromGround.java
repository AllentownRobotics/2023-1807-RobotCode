// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCMDS;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Utils.Enums.ClawState;
import frc.robot.Utils.Enums.WristState;
import frc.robot.commands.ArmCMDS.LowLevelCMDS.SetArmAngle;
import frc.robot.commands.ClawCMDS.LowLevelCMDS.SetClawState;
import frc.robot.commands.ClawCMDS.LowLevelCMDS.SetWristState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpFromGround extends SequentialCommandGroup {
  /** Creates a new PickUpFromGround. */
  public PickUpFromGround(RobotContainer rc) {
    addCommands(new SetArmAngle(rc.arm, 200.0),
                Commands.waitUntil(rc.arm::atSetPoint),
                new SetWristState(rc.claw, WristState.WristOut),
                new SetClawState(rc.claw, ClawState.Open),
                Commands.run(() -> rc.arm.runAtSpeed(0.025), rc.arm).until(rc.arm::atBumpers),
                new SetArmAngle(rc.arm, 320.0));
  }
}
