// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LED;
import frc.robot.Utils.Constants.*;

public class LightCMD extends InstantCommand {
  public static int animNumber;
  /** Creates a new LEDCommand. */
  public LightCMD(int animNumber) {
    LED.animNumber = animNumber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.light);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (animNumber) {
      case AnimNumberConstants.IDLE_ANIM_NUMBER: LED.IdleAnim();
      break;
      case AnimNumberConstants.CONE_REQ_ANIM_NUMBER: LED.ConeReqAnim();
      break;
      case AnimNumberConstants.CUBE_REQ_ANIM_NUMBER: LED.CubeReqAnim();
      break;
      case AnimNumberConstants.CONE_TRANSPORT_ANIM_NUMBER: LED.ConeTransportAnim();
      break;
      case AnimNumberConstants.CUBE_TRANSPORT_ANIM_NUMBER: LED.CubeTransportAnim();
      break;
      case AnimNumberConstants.CONE_SCORE_ANIM_NUMBER: LED.ConeScoreAnim(); 
      break;
      case AnimNumberConstants.CUBE_SCORE_ANIM_NUMBER: LED.CubeScoreAnim();
      break;
      case AnimNumberConstants.ENDGAME_ANIM_NUMBER: LED.EndGameAnim();
      break;
      case AnimNumberConstants.RESET_ANIM_NUMBER: LED.NoAnim();
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
    //RobotContainer.light.NoAnim();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
