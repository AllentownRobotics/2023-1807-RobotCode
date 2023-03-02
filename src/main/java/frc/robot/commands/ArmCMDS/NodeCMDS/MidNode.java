// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCMDS.NodeCMDS;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Enums.WristState;
import frc.robot.commands.ArmCMDS.WaitForPlace;
import frc.robot.commands.ArmCMDS.LowLevelCMDS.SetArmAngle;
import frc.robot.commands.ClawCMDS.LowLevelCMDS.SetWristState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidNode extends SequentialCommandGroup {
    public MidNode(Arm arm, Claw claw, CommandXboxController controller) {
        addCommands(new SetWristState(claw, WristState.WristOut),
                    new WaitForPlace(arm, new SetArmAngle(arm, ArmConstants.ANGLE_CONE_MID, ArmConstants.ANGLE_CUBE_MID), controller));
      }
}
