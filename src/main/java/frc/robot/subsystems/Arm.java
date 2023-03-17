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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Constants.ClawConstants;
import frc.robot.Utils.Enums.ClawState;
import frc.robot.Utils.Enums.PlacementType;

public class Arm extends SubsystemBase {
  CANSparkMax leftMotor = new CANSparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);

  CANSparkMax rightMotor = new CANSparkMax(ArmConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

  AbsoluteEncoder encoder;

  SparkMaxPIDController pidController;

  double desiredAngle;

  PlacementType placeType;

  Claw claw;

  public Arm(Claw claw) {
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    encoder = ArmConstants.USE_LEFT_ENCODER ? leftMotor.getAbsoluteEncoder(Type.kDutyCycle) : rightMotor.getAbsoluteEncoder(Type.kDutyCycle); 
    encoder.setInverted(!ArmConstants.USE_LEFT_ENCODER);
    encoder.setPositionConversionFactor(360.0);
    encoder.setVelocityConversionFactor(encoder.getPositionConversionFactor() / 60.0);
    
    pidController = leftMotor.getPIDController();
    pidController.setFeedbackDevice(encoder);

    // Set PID values from SysID
    pidController.setP(ArmConstants.PID_kP);
    pidController.setI(ArmConstants.PID_kI);
    pidController.setD(ArmConstants.PID_kD);
    pidController.setFF(ArmConstants.PID_kFF);
    pidController.setOutputRange(-0.3, 0.3);
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

    placeType = PlacementType.Cone;
  }

  @Override
  public void periodic() {
    pidController.setReference(desiredAngle, ControlType.kPosition);

    SmartDashboard.putNumber("Arm Angle", encoder.getPosition());
    SmartDashboard.putNumber("Set Point", desiredAngle);
  }

  /**
   * Gets the current angle of the arm
   * @return the angle of the arm
   */
  public double getAngle(){
    return encoder.getPosition();
  }

  /**
   * Gets the desired angle of the arm
   * @return the desired angle of the arm
   */
  public double getDesiredAngle(){
    return desiredAngle;
  }

  /**
   * Sets the angle the arm will attempt to go to
   * @param angle The desired angle of the arm
   */
  public void setDesiredAngle(double angle) {
    desiredAngle = angle;
  }

  /**
   * Sets the desired angle to the current angle shifted by the given number of degrees
   * @param degrees Amount to change the current angle by
   */
  public void rotateBy(double degrees){
    desiredAngle = encoder.getPosition() + degrees;
  }

  /**
   * Toggles the wrist being straightened out
   */
  public void toggleWrist(){
    claw.toggleWristState();
  }

  /**
   * Gets the type of placement to be used true for cones and false for cubes
   * @return The type of placement to use
   */
  public PlacementType getPlaceType(){
    return placeType;
  }

  /**
   * Toggles the placement type between cone and cube
   */
  public void togglePlacementType(){
    if (placeType == PlacementType.Cone){
      placeType = PlacementType.Cube;
      return;
    }
    placeType = PlacementType.Cone;
  }

  /**
   * Sets the placement type
   * @param placementType placement type to set to
   */
  public void setPlaceType(PlacementType placementType){
    placeType = placementType;
  }

  /**
   * Returns false of the claw is currently closed and true if it is currently open
   * Best used as a boolean supplier for a {@code waitUntil} command
   * @return Inverse of the claws current state
   */
  public boolean getNOTHolding(){
    if (claw.clawState.equals(ClawState.Open)){
      return true;
    }
    return false;
  }

  /**
   * Checks whether or not the arm is currently at the desired angle within a given tolerance defined in the constants.
   * Best used as a boolean supplier with a {@code waitUntil} command
   * @return If the arm is at the desired angle
   */
  public boolean atSetPoint(){
    if (Math.abs(encoder.getPosition() - desiredAngle) <= 4.5){
      return true;
    }
    return false;
  }

  /**
   * Checks whether or not the arm is currently at the desired reset angle whithin a given tolerance defined in the constants.
   * Best used as a boolean supplier with a {@code waitUntil} command
   * @return If the arm is at the desired reset angle
   */
  public boolean atReset(){
    if (encoder.getPosition() <= 15.0){
      desiredAngle = 14.2;
      return true;
    }
    return false;
  }

  /**
   * Checks whether or not the arm is currently at desired bumper angle within a given tolerance defined in the constants.
   * Best used as a boolean supplier with a {@code waitUntil} command
   * @return If the arm is at the desired bumpers angle
   */
  public boolean atBumpers(){
    if (encoder.getPosition() >= 290.0){
      desiredAngle = 300.0;
      return true;
    }
    return false;
  }

  /**
   * Checks whether or not the arm has passed the desired wrist flip point within a given tolerance defined in the constants.
   * Best used as a boolean supplier with a {@code waitUntil} command
   * @return If the arm is passed the flip point
   */
  public boolean isWristAllowedOut(){
    boolean minCheck = encoder.getPosition() >= ClawConstants.ANGLE_WRIST_EXCLUSIONZONE_MIN;
    boolean maxCheck = encoder.getPosition() <= ClawConstants.ANGLE_WRIST_EXCLUSIONZONE_MAX;
    
    return !(minCheck && maxCheck);
  }

  /**
   * Sets the motors to run at the given percent speed
   * @param percentSpeed the speed for the motors to run at
   */
  public void runAtSpeed(double percentSpeed){
    leftMotor.set(percentSpeed);
  }

  public void setBrakes(IdleMode mode){
    leftMotor.setIdleMode(mode);
    rightMotor.setIdleMode(mode);
  }
}