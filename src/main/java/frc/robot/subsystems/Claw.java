// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 
import frc.robot.Utils.Constants.ClawConstants;
import frc.robot.Utils.Enums.ClawState;
import frc.robot.Utils.Enums.WristState;

public class Claw extends SubsystemBase {
  DoubleSolenoid wristPiston = new DoubleSolenoid(ClawConstants.WRIST_ID, PneumaticsModuleType.REVPH, 
  ClawConstants.WRIST_CHANNEL_FORWARD, ClawConstants.WRIST_CHANNEL_BACKWARD);
  DoubleSolenoid clawPiston = new DoubleSolenoid(ClawConstants.CLAW_ID, PneumaticsModuleType.REVPH, 
  ClawConstants.CLAW_CHANNEL_FORWARD, ClawConstants.CLAW_CHANNEL_BACKWARD);

  DigitalInput sensor = new DigitalInput(0);

  WristState wristState = WristState.WristOut;
  ClawState clawState = ClawState.Closed;

  boolean manualWristControlAllowed = true;
  boolean allowAutoGrab = false;

  static Claw instance = null;

  /**
   * Creates a new Claw. 
   * NOTE: This method should not be manually called. Instead,
   * use the singleton instance by calling the static method {@link Claw#getInstance()} 
   */
  public Claw() {}

  /**
   * Gets the singleton instance of the claw. If no instance exists one is automatically created.
   * @return The singleton instance of the claw
   */
  public static Claw getInstance(){
    if (instance == null){
      instance = new Claw();
    }
    return instance;
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

  /**
   * Sets whether or not the wrist should be in the user provided state or be down to prevent breaking extension limit
   * @param allowed Whether or not the wrist should listen to manual control
   */
  public void setManualWristControlAllowed(boolean allowed){
    manualWristControlAllowed = allowed;
  }

  /**
   * Checks if the wirst is currently authorized to follow user control. Returns true if so and false otherwise
   * @return Whether or not the wirst is currently authorized to follow user control
   */
  public boolean isManualWristControlAuthorized(){
    return manualWristControlAllowed;
  }

  /**
   * Sets the state of the automatic grab mode. If true the claw will automatically close when a game piece is detected
   * @param allowed Whether automatic grabbing should be allowed
   */
  public void setAutoGrabAllowed(boolean allowed){
    allowAutoGrab = allowed;
  }

  /**
   * Checks if there is a game piece in the grab range and, if allowed, will return true to automatically close.
   * Set the automatic grab permission using {@link Claw#setAutoGrabAllowed(boolean)}
   * @return If the claw should automatically close
   */
  public boolean shouldAutoClose(){
    return allowAutoGrab && !sensor.get();
  }

  @Override
  public void periodic() {
    if (manualWristControlAllowed){
      wristPiston.set(wristState.equals(WristState.WristOut) ? Value.kReverse : Value.kForward);
    }
    else{
      wristPiston.set(Value.kForward);
    }

    clawPiston.set(clawState.equals(ClawState.Closed) ? Value.kForward : Value.kReverse);

    SmartDashboard.putBoolean("Auto Grab", allowAutoGrab);
  }
}
