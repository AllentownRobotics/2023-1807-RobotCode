// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 
import frc.robot.Utils.Constants.ClawConstants;

public class Claw extends SubsystemBase {
  DoubleSolenoid wristPiston = new DoubleSolenoid(ClawConstants.WRIST_ID, PneumaticsModuleType.REVPH, 
    ClawConstants.WRIST_CHANNEL_FORWARD, ClawConstants.WRIST_CHANNEL_BACKWARD);
  DoubleSolenoid clawPiston = new DoubleSolenoid(ClawConstants.CLAW_ID, PneumaticsModuleType.REVPH, 
    ClawConstants.CLAW_CHANNEL_FORWARD, ClawConstants.CLAW_CHANNEL_BACKWARD);
  
  boolean wristOut = false;
  boolean holding = false;

  public Claw() {
  }

  public boolean getHolding(){
    return holding;
  }

  public boolean getWristStraight(){
    return wristOut;
  }

  public void setHolding(boolean hold){
    holding = hold;
  }

  public void setWristOut(boolean wristStraight){
    wristOut = wristStraight;
  }

  public void toggleHold(){
    holding = !holding;
  }

  public void toggleWrist(){
    wristOut = !wristOut;
  }

  // Handles setting piston position
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("holding", holding);
    SmartDashboard.putBoolean("wrist out", wristOut);

    wristPiston.set(wristOut ? Value.kReverse : Value.kForward);
    clawPiston.set(holding ? Value.kForward : Value.kReverse);
  }


}
