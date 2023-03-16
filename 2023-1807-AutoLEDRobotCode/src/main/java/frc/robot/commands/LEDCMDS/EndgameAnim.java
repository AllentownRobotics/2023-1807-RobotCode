// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDCMDS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;

public class EndgameAnim extends CommandBase {

  LED light;
  /** Creates a new EndgameAnim. */
  public EndgameAnim(LED light) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(light);
    this.light = light;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    light.setAllianceColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    light.NotBalancedAnim();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return light.CalculateTilt() <= 2.5;
  }
}
