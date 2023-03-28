// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCMDS;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCMDS.LowLevelCMDS.SetArmAngle;
import frc.robot.commands.ClawCMDS.WristToStandBy;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetArm extends ParallelCommandGroup {
  /**
   * Sequential command group which sets the arm back to its reset position with the claw in its standby position.
   * Ends after the arm is in the reset position and the claw is in standby position
   * @param rc Robotcontainer
   */
  public ResetArm(RobotContainer rc) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new WristToStandBy(), new SetArmAngle(rc.arm, 12.0));
  }
}
