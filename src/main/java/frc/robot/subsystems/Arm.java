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
import frc.robot.Utils.FauxTrapezoidProfile;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Constants.ClawConstants;
import frc.robot.Utils.FauxTrapezoidProfile.FeedForwardFeeder;

public class Arm extends SubsystemBase {
  CANSparkMax leftMotor = new CANSparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);

  CANSparkMax rightMotor = new CANSparkMax(ArmConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

  AbsoluteEncoder encoder;

  SparkMaxPIDController pidController;
  //PIDController pidController = new PIDController(ArmConstants.PID_kP, ArmConstants.PID_kI, ArmConstants.PID_kD);
  FauxTrapezoidProfile profile = new FauxTrapezoidProfile(210.0, 3.0 * 1.4, 1.0, 2.5);
  //ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0);

  Claw claw;

  double desiredAngle;
  double manualPercent;
  boolean automaticControl;

  public Arm(Claw claw) {
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    encoder = ArmConstants.USE_LEFT_ENCODER ? leftMotor.getAbsoluteEncoder(Type.kDutyCycle) : rightMotor.getAbsoluteEncoder(Type.kDutyCycle); 
    encoder.setInverted(!ArmConstants.USE_LEFT_ENCODER);
    encoder.setPositionConversionFactor(360.0);
    encoder.setVelocityConversionFactor(encoder.getPositionConversionFactor() / 60.0);

    leftMotor.setInverted(false);

    rightMotor.follow(leftMotor, true);

    pidController = leftMotor.getPIDController();
    pidController.setP(ArmConstants.PID_kP, 0);
    pidController.setI(ArmConstants.PID_kI, 0);
    pidController.setD(ArmConstants.PID_kD, 0);
    pidController.setFF(ArmConstants.PID_kFF, 0);
    pidController.setOutputRange(-0.5, 0.5, 0);

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    leftMotor.setSmartCurrentLimit(40);
    rightMotor.setSmartCurrentLimit(40);

    leftMotor.burnFlash();
    rightMotor.burnFlash();

    this.claw = claw;

    desiredAngle = 0.0;

    profile.setGoal(0.0);

    manualPercent = 0.0;

    automaticControl = true;
  }

  @Override
  public void periodic() {
    FeedForwardFeeder profileOutput = profile.calculate(encoder.getPosition());
    if (automaticControl){
      //double pidOutput = pidController.calculate(encoder.getVelocity(), profileOutput.velocity);
      //double ffOutput = feedforward.calculate(profileOutput.position, profileOutput.velocity, profileOutput.acceleration);

      pidController.setReference(profileOutput.velocity, ControlType.kVelocity, 0);
    }
    else{
      leftMotor.set(manualPercent);
    }

    SmartDashboard.putNumber("Calculated Velocity", profileOutput.velocity);
    SmartDashboard.putNumber("Arm Angle", encoder.getPosition());
    SmartDashboard.putNumber("Arm Velocity", encoder.getVelocity());
    SmartDashboard.putNumber("Set Point", desiredAngle);
  }

  /**
   * Sets the desired arm angle to the provided angle
   * @param desiredAngle New desired angle
   */
  public void setDesiredAngle(double desiredAngle){
    this.desiredAngle = desiredAngle;
    profile.setGoal(desiredAngle);
  }

  /**
   * Returns the positional error between the desired angle and the current angle
   * @return {@code desired - current}
   */
  public double getPositionalError(){
    return desiredAngle - encoder.getPosition();
  }

  public boolean atSetPoint(){
    return Math.abs(getPositionalError()) <= ArmConstants.ANGLE_CHECKTOLERANCE_DEGREES;
  }

  public double getArmAngle(){
    return encoder.getPosition();
  }

  public boolean isWristAllowedOut(){
    return !(encoder.getPosition() >= ClawConstants.ANGLE_WRIST_EXCLUSIONZONE_MIN && encoder.getPosition() <= ClawConstants.ANGLE_WRIST_EXCLUSIONZONE_MAX);
  }

  public boolean atReset(){
    return encoder.getPosition() <= 15.0;
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

  /**
   * Get the Claw subsystem contained within this Arm
   * @return The contained Claw
   */
  public Claw getClawSubsystem(){
    return claw;
  }
}