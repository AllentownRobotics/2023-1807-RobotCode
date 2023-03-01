// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.CompressorConstants;
import frc.robot.Enums.ClawState;
import frc.robot.Enums.WristState;

public class Claw extends SubsystemBase {

  DoubleSolenoid wristPiston = new DoubleSolenoid(CompressorConstants.compressorCanId, PneumaticsModuleType.REVPH, 
    ClawConstants.wristForwardChannel, ClawConstants.wristReverseChannel);
  DoubleSolenoid clawPiston = new DoubleSolenoid(CompressorConstants.compressorCanId, PneumaticsModuleType.REVPH, 
    ClawConstants.clawForwardChannel, ClawConstants.clawReverseChannel);
  
  WristState wristState = WristState.WristOut;
  ClawState clawState = ClawState.Closed;

  public Claw() {
  }

  /**
   * Gets the current state of the claw
   * @return The state of the claw
   */
  public ClawState getClawState(){
    return clawState;
  }

  /**
   * Gets the current state of the wrist
   * @return The state of the wrist
   */
  public WristState getWristState(){
    return wristState;
  }

  /**
   * Sets the claw state
   * @param state State for the claw to be changed to
   */
  public void setClawState(ClawState state){
    clawState = state;
  }

  /**
   * Sets the wrist state
   * @param state State for the wrist to be changed to
   */
  public void setWristState(WristState state){
    wristState = state;
  }

  /**
   * Toggles the current state of the claw
   */
  public void toggleClawState(){
    if (clawState.equals(ClawState.Open)){
      clawState = ClawState.Closed;
      return;
    }
    clawState = ClawState.Open;
  }
  
  /**
   * Toggles the current state of the wrist
   */
  public void toggleWristState(){
    if (wristState.equals(WristState.WristOut)){
      wristState = WristState.WristDown;
      return;
    }
    wristState = WristState.WristOut;
  }

  @Override
  public void periodic() {
    wristPiston.set(wristState.equals(WristState.WristOut) ? Value.kReverse : Value.kForward);
    clawPiston.set(clawState.equals(ClawState.Closed) ? Value.kForward : Value.kReverse);
  }
}
