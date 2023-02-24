// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;

public class Collector extends SubsystemBase {
  CANSparkMax topMotor = new CANSparkMax(CollectorConstants.TOP_MOTOR_ID, MotorType.kBrushless);
  CANSparkMax bottomMotor = new CANSparkMax(CollectorConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);

  DoubleSolenoid piston = new DoubleSolenoid(CollectorConstants.COLLECTOR_PISTON_ID, PneumaticsModuleType.REVPH, 
    CollectorConstants.COLLECTOR_PISTON_CHANNEL_FORWARD, CollectorConstants.COLLECTOR_PISTON_CHANNEL_BACKWARD);

  // SUBJECT TO CHANGE
  CANSparkMax beltMotor = new CANSparkMax(CollectorConstants.BELT_MOTOR_ID, MotorType.kBrushless);

  public Collector() {
    topMotor.setInverted(true);
    bottomMotor.setInverted(false);

    topMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);    
  
    topMotor.burnFlash();
    bottomMotor.burnFlash();
    
    beltMotor.setInverted(false);
    beltMotor.setIdleMode(IdleMode.kCoast);
    beltMotor.burnFlash();
  }

  public void Collect(){
    piston.set(Value.kForward);

    topMotor.set(CollectorConstants.COLLECTOR_SPEED_MAX);
    bottomMotor.set(CollectorConstants.COLLECTOR_SPEED_MAX);

    beltMotor.set(CollectorConstants.COLLECTOR_SPEED_MAX);
  }

  public void Retract(){
    piston.set(Value.kReverse);

    topMotor.set(0.0);

    bottomMotor.set(0.0);

    beltMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
