// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Enums.WristState;
import frc.robot.commands.Arm.LowLevelCommands.SetArmAngle;
import frc.robot.commands.Claw.SetWristToStandBy;
import frc.robot.commands.Claw.LowLevelCommands.SetWristState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetArm extends SequentialCommandGroup {
  /**
   * Sequential command group which sets the arm back to its reset position with the claw in its standby position.
   * Ends after the arm is in the reset position and the claw is in standby position
   * @param rc Robotcontainer
   */
  public ResetArm(RobotContainer rc) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetWristState(rc.m_Claw, WristState.WristDown),
                new SetArmAngle(rc.m_Arm, 40.0), 
                Commands.waitUntil(rc.m_Arm::atSetPoint),  
                new ParallelCommandGroup(Commands.run(() -> rc.m_Arm.runAtSpeed(-0.025), rc.m_Arm).until(rc.m_Arm::atReset), 
                                          new SetWristToStandBy(rc.m_Claw)));
  }
}
