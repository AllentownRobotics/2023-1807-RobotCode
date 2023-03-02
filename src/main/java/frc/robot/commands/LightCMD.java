// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Utils.Constants.AnimNumberConstants;
import frc.robot.subsystems.LED;

public class LightCMD extends CommandBase {
  public int animNumber;

  private LED led;
  // 0 = red, 1 = green, 2 = blue, 3 = rainbow, 4 = blink, 5 = none
  /** Creates a new LEDCommand. */
  public LightCMD(LED led, int animNumber) {
    this.animNumber = animNumber;

    this.led = led;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (animNumber) {
        case AnimNumberConstants.IDLE_ANIM_NUMBER: led.IdleAnim();
        break;
        case AnimNumberConstants.CONE_REQ_ANIM_NUMBER: led.ConeReqAnim();
        break;
        case AnimNumberConstants.CUBE_REQ_ANIM_NUMBER: led.CubeReqAnim();
        break;
        case AnimNumberConstants.CONE_TRANSPORT_ANIM_NUMBER: led.ConeTransportAnim();
        break;
        case AnimNumberConstants.CUBE_TRANSPORT_ANIM_NUMBER: led.CubeTransportAnim();
        break;
        case AnimNumberConstants.CONE_SCORE_ANIM_NUMBER: led.ConeScoreAnim(); 
        break;
        case AnimNumberConstants.CUBE_SCORE_ANIM_NUMBER: led.CubeScoreAnim();
        break;
        case AnimNumberConstants.ENDGAME_ANIM_NUMBER: led.EndGameAnim();
        break;
        case AnimNumberConstants.RESET_ANIM_NUMBER: led.NoAnim();
        break;
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    led.NoAnim();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
