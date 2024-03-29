// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCMDS.NodeCMDS;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Utils.LightAnimation;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Constants.LightsConstants;
import frc.robot.Utils.Enums.GamePiece;
import frc.robot.Utils.Enums.WristState;
import frc.robot.commands.ArmCMDS.WaitForPlace;
import frc.robot.commands.ArmCMDS.LowLevelCMDS.SetArmAngle;
import frc.robot.commands.ClawCMDS.LowLevelCMDS.SetWristState;
import frc.robot.commands.LightCMDS.BlinkColor;
import frc.robot.commands.LightCMDS.SetAnimation;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HighNode extends SequentialCommandGroup {
 /**
   * Sequential command group that sets the wrist state out then actuates the arm for placement at high nodes.
   * Ends after the currently held game piece is dropped
   * @param arm Arm subsystem
   * @param claw Claw subsystem
   * @param controller Operator controller
   */
  public HighNode(Arm arm, Claw claw) {
    addCommands(new SetWristState(WristState.WristOut),
                new WaitForPlace(arm, new SetArmAngle(arm, ArmConstants.ANGLE_CONE_HIGH)).deadlineWith(
                  new BlinkColor(
                    () -> (RobotContainer.desiredGamePiece.equals(GamePiece.Cone) ? LightsConstants.COLOR_CONE : LightsConstants.COLOR_CUBE), 
                    0.1, 0.1)),
                new SetAnimation(LightAnimation.nullAnim));
  }
}
