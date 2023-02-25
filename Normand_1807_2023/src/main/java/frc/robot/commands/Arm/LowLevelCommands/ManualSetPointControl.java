// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.LowLevelCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ManualSetPointControl extends CommandBase {
  Arm arm;
  CommandXboxController controller;

  /** Creates a new ManualSetPointControl. */
  public ManualSetPointControl(Arm arm, CommandXboxController controller) {
    addRequirements(arm);

    this.arm = arm;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //arm.rotateBy(ArmConstants.ANGLE_MANUAL_INPUT_MODIFIER * MathUtil.applyDeadband(-controller.getLeftY(), 0.3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
