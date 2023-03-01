// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.LowLevelCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Enums.PlacementType;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArmAngle extends InstantCommand {
  Arm arm;

  double coneAngle;
  double cubeAngle;

  /**
   * Instant command which sets the arms desired angle then instantly ends.
   * NOTE: This command does not wait for the arm to be at the desired angle before ending
   * To achieve this use a {@code Commands.waitUntil} command 
   * passing in arm's {@code atSetPoint()} as a boolean supplier Ex. {@code Commands.waitUntil(arm::atSetPoint)}
   * @param arm Arm subsystem
   * @param coneAngle Angle for the arm to go to if a cone placement is desired
   * @param cubeAngle Angle for the arm to go to if a cube placement is desired
   */
  public SetArmAngle(Arm arm, double coneAngle, double cubeAngle) {
    addRequirements(arm);

    this.arm = arm;
    this.coneAngle = coneAngle;
    this.cubeAngle = cubeAngle;
  }

  /**
   * Instant command which sets the arms desired angle then instantly ends.
   * NOTE: This command does not wait for the arm to be at the desired angle before ending
   * To achieve this use a {@code Commands.waitUntil} command 
   * passing in arm's {@code atSetPoint()} as a boolean supplier Ex. {@code Commands.waitUntil(arm::atSetPoint)}
   * @param arm Arm subsystem
   * @param uniAngle Angle for the arm to go to regardless of the desired placement type
   */
  public SetArmAngle(Arm arm, double uniAngle){
    this(arm, uniAngle, uniAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double angle = arm.getPlaceType().equals(PlacementType.Cone) ? coneAngle : cubeAngle;
    arm.setDesiredAngle(angle);
  }
}
