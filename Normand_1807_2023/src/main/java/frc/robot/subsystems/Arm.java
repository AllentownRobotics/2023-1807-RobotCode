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
import frc.robot.LagBehind;
import frc.robot.Constants.ArmConstants;
import frc.robot.Enums.ClawState;
//import frc.robot.Constants.ClawConstants;
import frc.robot.Enums.PlacementType;

public class Arm extends SubsystemBase {
  CANSparkMax leftMotor = new CANSparkMax(ArmConstants.armMotorLeftCanId, MotorType.kBrushless);

  CANSparkMax rightMotor = new CANSparkMax(ArmConstants.armMotorRightCanId, MotorType.kBrushless);

  AbsoluteEncoder encoder;

  SparkMaxPIDController pidController;

  double desiredAngle;

  PlacementType placeType;

  Claw claw;

  LagBehind checkValues;

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
    pidController.setOutputRange(ArmConstants.armMinOutput,ArmConstants.armMaxOutput);
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

    //checkValues = new LagBehind(encoder.getPosition());

    placeType = PlacementType.Cone;
  }

  @Override
  public void periodic() {
    pidController.setReference(desiredAngle, ControlType.kPosition);

    //checkValues.update(encoder.getPosition());

    SmartDashboard.putNumber("Arm Angle", encoder.getPosition());
    SmartDashboard.putNumber("Set Point", desiredAngle);

    /*if (encoder.getPosition() < ClawConstants.ANGLE_WRIST_FLIPPOINT){
      claw.setWristOut(false);
    }*/
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
   * Shifts the desired angle by the given number of degrees
   * @param degrees Amount to change the desired angle by
   */
  public void rotateBy(double degrees){
    desiredAngle += degrees;
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
   * Best used as a boolean supplier for a waitUntil command
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
   * Best used as a boolean supplier with a waitUntil command
   * @return If the arm is at the desired angle
   */
  public boolean atSetPoint(){
    if (Math.abs(encoder.getPosition()-desiredAngle)<= 7){
      return true;
    }
    return false;
  }

  /**
   * Checks whether or not the arm is currently at the desired reset angle whithin a given tolerance defined in the constants.
   * Best used as a boolean supplier with a waitUntil command
   * @return If the arm is at the desired reset angle
   */
  public boolean atReset(){
    if (encoder.getPosition() <= 13.0){
      desiredAngle = 0.0;
      return true;
    }
    return false;
  }

  public boolean atBumpers(){
    if (encoder.getPosition() >= 290.0){
      desiredAngle = 300.0;
      return true;
    }
    return false;
  }

  /**
   * Sets the motors to run at the given percent speed
   * @param percentSpeed the speed for the motors to run at
   */
  public void runAtSpeed(double percentSpeed){
    leftMotor.set(percentSpeed);
  }
}
