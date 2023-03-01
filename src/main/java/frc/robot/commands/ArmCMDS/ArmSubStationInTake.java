// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCMDS;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCMDS.LowLevelCMDS.SetArmAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmSubStationInTake extends ParallelCommandGroup {
  /**
   * Parallel command group which runs 2 instant commands and ends instantly
   * Sets the claw to standby position and sets the arm to 35 degrees
   * @param rc
   */
  public ArmSubStationInTake(RobotContainer rc) {
    addCommands(new SetWristToStandBy(rc.claw),
                new SetArmAngle(rc.arm, 45.0));
  }
}
