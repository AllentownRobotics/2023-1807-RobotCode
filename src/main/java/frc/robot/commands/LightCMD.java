// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Utils.Constants.AnimNumberConstants;

public class LightCMD extends CommandBase {
  public int animNumber;
  // 0 = red, 1 = green, 2 = blue, 3 = rainbow, 4 = blink, 5 = none
  /** Creates a new LEDCommand. */
  public LightCMD(int animNumber) {
    this.animNumber = animNumber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.light);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*switch (animNumber) {
        case AnimNumberConstants.IDLE_ANIM_NUMBER: RobotContainer.light.IdleAnim();
        break;
        case AnimNumberConstants.CONE_REQ_ANIM_NUMBER: RobotContainer.light.ConeReqAnim();
        break;
        case AnimNumberConstants.CUBE_REQ_ANIM_NUMBER: RobotContainer.light.CubeReqAnim();
        break;
        case AnimNumberConstants.CONE_TRANSPORT_ANIM_NUMBER: RobotContainer.light.ConeTransportAnim();
        break;
        case AnimNumberConstants.CUBE_TRANSPORT_ANIM_NUMBER: RobotContainer.light.CubeTransportAnim();
        break;
        case AnimNumberConstants.CONE_SCORE_ANIM_NUMBER: RobotContainer.light.ConeScoreAnim(); 
        break;
        case AnimNumberConstants.CUBE_SCORE_ANIM_NUMBER: RobotContainer.light.CubeScoreAnim();
        break;
        case AnimNumberConstants.ENDGAME_ANIM_NUMBER: RobotContainer.light.EndGameAnim();
        break;
        case AnimNumberConstants.RESET_ANIM_NUMBER: RobotContainer.light.NoAnim();
        break;
      }*/
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.light.NoAnim();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
