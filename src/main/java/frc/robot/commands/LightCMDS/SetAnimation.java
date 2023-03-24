// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LightCMDS;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Utils.LightAnimation;
import frc.robot.subsystems.Lights;

public class SetAnimation extends InstantCommand {
  public LightAnimation animation;

  /**
   * See {@code LightAnimation} class for ID key
   */
  public SetAnimation(LightAnimation animation) {
    this.animation = animation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Lights.getInstance().setAnimation(animation);
  }
}
