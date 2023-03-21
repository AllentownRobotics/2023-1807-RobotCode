// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCMDS.LowLevelCMDS;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArmAngle extends InstantCommand {
  Arm arm;

  double angle;

  /**
   * Instant command which sets the arms desired angle then instantly ends.
   * NOTE: This command does not wait for the arm to be at the desired angle before ending
   * To achieve this use a {@code Commands.waitUntil} command 
   * passing in arm's {@code atSetPoint()} as a boolean supplier Ex. {@code Commands.waitUntil(arm::atSetPoint)}
   * @param arm Arm subsystem
   * @param uniAngle Angle for the arm to go to
   */
  public SetArmAngle(Arm arm, double angle){
    addRequirements(arm);

    this.arm = arm;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setDesiredAngle(angle);
  }
}
