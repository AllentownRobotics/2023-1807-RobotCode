// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCMDS;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCMDS.LowLevelCMDS.SetArmAngle;
import frc.robot.commands.ClawCMDS.WristToStandBy;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundPickup extends SequentialCommandGroup {
  /**
   * Sequential command group which puts the arm in position to pick up a game piece from the ground.
   * NOTE: this command does not handle closing the claw to grab the game piece.
   * Ends once the arm is either at it's set point or it times out after 2 seconds
   * @param rc
   */
  public GroundPickup(RobotContainer rc) {
    addCommands(Commands.parallel(new SetArmAngle(rc.arm, 300.0), new WristToStandBy()),
      Commands.waitUntil(rc.arm::atSetPoint).withTimeout(2.0));
  }
}
