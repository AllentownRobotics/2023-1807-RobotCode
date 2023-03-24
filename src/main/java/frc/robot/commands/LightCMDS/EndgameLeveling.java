// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LightCMDS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils.LightAnimation;
import frc.robot.subsystems.Lights;

public class EndgameLeveling extends CommandBase {
  Lights lights;

  /**
   * Command which clears all current animations then starts the endgame leveling animation.
   * Runs until interrupted
   */
  public EndgameLeveling() {
    lights = Lights.getInstance();
    addRequirements(lights);
    withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lights.setAnimation(LightAnimation.endgame);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
