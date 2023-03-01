// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Enums.ClawState;
import frc.robot.Enums.WristState;
import frc.robot.commands.Arm.LowLevelCommands.SetArmAngle;
import frc.robot.commands.Claw.LowLevelCommands.SetClawState;
import frc.robot.commands.Claw.LowLevelCommands.SetWristState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpFromGround extends SequentialCommandGroup {
  /** Creates a new PickUpFromGround. */
  public PickUpFromGround(RobotContainer rc) {
    addCommands(new SetArmAngle(rc.m_Arm, 200.0),
                Commands.waitUntil(rc.m_Arm::atSetPoint),
                new SetWristState(rc.m_Claw, WristState.WristOut),
                new SetClawState(rc.m_Claw, ClawState.Open),
                Commands.run(() -> rc.m_Arm.runAtSpeed(0.025), rc.m_Arm).until(rc.m_Arm::atBumpers),
                new SetArmAngle(rc.m_Arm, 320.0));
  }
}
