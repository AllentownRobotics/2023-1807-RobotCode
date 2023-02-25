// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
//import frc.robot.Constants.ClawConstants;

public class Arm extends SubsystemBase {
  CANSparkMax leftMotor = new CANSparkMax(ArmConstants.armMotorLeftCanId, MotorType.kBrushless);

  CANSparkMax rightMotor = new CANSparkMax(ArmConstants.armMotorRightCanId, MotorType.kBrushless);

  AbsoluteEncoder encoder;

  SparkMaxPIDController pidController;

  double desiredAngle;

  boolean placeCone;

  Claw claw;

  public Arm(Claw claw) {
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    // Determine which encoder to use
    encoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle); 
    // If using right encoder set inversion false, if not set true
    encoder.setInverted(false);
    // Set conversion factor to output in degrees and degrees/sec
    encoder.setPositionConversionFactor(360.0);
    encoder.setVelocityConversionFactor(encoder.getPositionConversionFactor() / 60.0);
    
    pidController = leftMotor.getPIDController();
    pidController.setFeedbackDevice(encoder);

    // Set PID values from SysID
    pidController.setP(ArmConstants.armP);
    pidController.setI(ArmConstants.armI);
    pidController.setD(ArmConstants.armD);
    pidController.setFF(ArmConstants.armFF);
    pidController.setOutputRange(-0.25,0.25);
    pidController.setPositionPIDWrappingEnabled(false);

    leftMotor.setInverted(false);

    // Have all motors follor master
    rightMotor.follow(leftMotor, true);

    // Set all motors to brake
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    leftMotor.setSmartCurrentLimit(40);
    rightMotor.setSmartCurrentLimit(40);

    leftMotor.burnFlash();
    rightMotor.burnFlash();

    this.claw = claw;

    desiredAngle = 0.0;
  }

  @Override
  public void periodic() {
    pidController.setReference(desiredAngle, ControlType.kPosition);

    SmartDashboard.putNumber("Arm Angle", encoder.getPosition());
    SmartDashboard.putNumber("Set Point", desiredAngle);

    /*if (encoder.getPosition() < ClawConstants.ANGLE_WRIST_FLIPPOINT){
      claw.setWristOut(false);
    }*/
  }

  public double getAngle(){
    return encoder.getPosition();
  }

  public double getDesiredAngle(){
    return desiredAngle;
  }

  public void setDesiredAngle(double angle) {
    desiredAngle = angle;
  }

  public void rotateBy(double degrees){
    desiredAngle += degrees;
  }

  public void toggleWrist(){
    claw.toggleWrist();
  }

  public boolean getConePlace(){
    return placeCone;
  }

  public void toggleConePlacing(){
    placeCone = !placeCone;
  }

  public void setConePlacing(boolean conePlacing){
    placeCone = conePlacing;
  }

  public boolean getNOTHolding(){
    return !claw.holding;
  }

  public boolean atSetPoint(){
    double error = Math.abs(encoder.getPosition() - desiredAngle); 
    if (error <= 7){
      return true;
    }
    return false;
  }

  public boolean atReset(){
    if (encoder.getPosition() <= 11.5){
      desiredAngle = 0.0;
      //claw.setWristOut(false);
      return true;
    }
    return false;
  }

  public void setHolding(boolean bool)
  {
    claw.setHolding(bool);
  }
  public void runAtSpeed(double percentSpeed){
    leftMotor.set(percentSpeed);
  }
}
