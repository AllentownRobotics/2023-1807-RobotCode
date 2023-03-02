// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {


  private CANSparkMax spindexMotor;
  CANSparkMax motor = new CANSparkMax(SpindexerConstants.SPINDEXER_MOTOR_ID, MotorType.kBrushless);

  RelativeEncoder encoder;

  SparkMaxPIDController pidController;

  double desriedRotations;
  
  public Spindexer() {
    motor.setIdleMode(IdleMode.kBrake);

    encoder = motor.getEncoder();
    encoder.setPosition(0);
    encoder.setPositionConversionFactor(1.0 / SpindexerConstants.SPINDEXER_GEARING_MOTORTOTABLE);
    encoder.setVelocityConversionFactor(encoder.getPositionConversionFactor() / 60.0);

    pidController = motor.getPIDController();
    pidController.setFeedbackDevice(encoder);
    pidController.setP(SpindexerConstants.SPINDEXER_P);
    pidController.setI(SpindexerConstants.SPINDEXER_I);
    pidController.setD(SpindexerConstants.SPINDEXER_D);
    pidController.setFF(SpindexerConstants.SPINDEXER_FF);
    pidController.setPositionPIDWrappingEnabled(false);
    pidController.setOutputRange(-SpindexerConstants.SPINDEXER_MOTOR_MAXOUTPUT, SpindexerConstants.SPINDEXER_MOTOR_MAXOUTPUT);

    motor.burnFlash();

    desriedRotations = 0.0;
  }

  @Override
  public void periodic() {
    pidController.setReference(desriedRotations, ControlType.kPosition);
  }

  /**
   * Resets the encoder to zero.
   * Recommend setting desired rotations to 0 afterwards to avoid unwanted movement
   */
  public void zeroEncoder(){
    encoder.setPosition(0.0);
  }

  /**
   * Runs the motor at the max allowed speed with the direction
   * @param direction the direction for the spindexer to spin 
   */
  public void spindex(double direction){
    motor.set(direction * SpindexerConstants.SPINDEXER_MOTOR_MAXOUTPUT);
  }

  /**
   * Checks if the table is at the specified rotations returns true if it is and false if not.
   * @return Whether or not the table is at the specified rotations
   */
  public boolean atSetPoint(){
    double error = Math.abs(encoder.getPosition() - desriedRotations);
    if (error <= 0.01){
      return true;
    }
    return false;
  }

  /**
   * Sets the desired rotations for the spindexer to spin
   * @param rotations the desired rotations to spin
   */
  public void setDesiredRotation(double rotations) {
    desriedRotations = rotations;
  }
}
 
