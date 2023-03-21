// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.ConversionLambda;
import frc.robot.Utils.FauxTrapezoidProfile;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Constants.ClawConstants;
import frc.robot.Utils.FauxTrapezoidProfile.FeedForwardFeeder;

public class Arm extends SubsystemBase {
  CANSparkMax leftMotor = new CANSparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);

  CANSparkMax rightMotor = new CANSparkMax(ArmConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

  AbsoluteEncoder encoder;

  PIDController pidController = new PIDController(ArmConstants.PID_kP, ArmConstants.PID_kI, ArmConstants.PID_kD);
  FauxTrapezoidProfile profile = new FauxTrapezoidProfile(210.0, 840.0, 1.0, 0.5);
  ArmFeedforward feedforward = new ArmFeedforward(0,0,0);
  Claw claw;

  double desiredAngle;
  boolean automaticControl;

  public Arm(Claw claw) {
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    encoder = ArmConstants.USE_LEFT_ENCODER ? leftMotor.getAbsoluteEncoder(Type.kDutyCycle) : rightMotor.getAbsoluteEncoder(Type.kDutyCycle); 
    encoder.setInverted(!ArmConstants.USE_LEFT_ENCODER);
    encoder.setPositionConversionFactor(360.0);
    encoder.setVelocityConversionFactor(encoder.getPositionConversionFactor() / 60.0);

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

    automaticControl = true;
  }

  @Override
  public void periodic() {
    FeedForwardFeeder profileOutput = profile.calculate(encoder.getPosition());
    double pidOutput = pidController.calculate(encoder.getVelocity(), profileOutput.velocity);
    profileOutput.convert(ConversionLambda.degreesToRadians);
    double ffOutput = feedforward.calculate(profileOutput.position, profileOutput.velocity, profileOutput.acceleration);

    voltageMotorControl(pidOutput + ffOutput);

    SmartDashboard.putNumber("Arm Angle", encoder.getPosition());
    SmartDashboard.putNumber("Set Point", desiredAngle);
  }

  public void setDesiredAngle(double desiredAngle){
    this.desiredAngle = desiredAngle;
  }

  public double getPositionalError(){
    return desiredAngle - encoder.getPosition();
  }

  public boolean atSetPoint(){
    return getPositionalError() <= ArmConstants.ANGLE_CHECKTOLERANCE_DEGREES;
  }

  public double getArmAngle(){
    return encoder.getPosition();
  }

  public boolean isWristAllowedOut(){
    return !(encoder.getPosition() >= ClawConstants.ANGLE_WRIST_EXCLUSIONZONE_MIN && encoder.getPosition() <= ClawConstants.ANGLE_WRIST_EXCLUSIONZONE_MAX);
  }

  public void openLoopMotorControl(double percentOutput){
    leftMotor.set(percentOutput);
  }

  public void voltageMotorControl(double volts){
    leftMotor.setVoltage(volts);
  }

  public void setBrakes(IdleMode mode){
    leftMotor.setIdleMode(mode);
    rightMotor.setIdleMode(mode);
  }
}