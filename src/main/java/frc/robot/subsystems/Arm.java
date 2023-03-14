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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
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

  PWMSparkMax motor = new PWMSparkMax(0);

  double desiredAngle;

  PlacementType placeType;

  Claw claw;

  Mechanism2d armMechanism = new Mechanism2d(2 * Units.inchesToMeters(ArmConstants.ARM_LENGTH_INCHES) + Units.inchesToMeters(23) + 1.0, 
                                              Units.inchesToMeters(ArmConstants.HEIGHT_OFFSET_FROM_GROUND_INCHES));
  MechanismRoot2d armRoot = armMechanism.getRoot("Arm", 
                                            Units.inchesToMeters(ArmConstants.ARM_LENGTH_INCHES) + Units.inchesToMeters(23) + 0.5, 
                                            0.0);

  MechanismLigament2d armUprights = armRoot.append(new MechanismLigament2d("Uprights", 
                                    Units.inchesToMeters(ArmConstants.HEIGHT_OFFSET_FROM_GROUND_INCHES),
                                    90.0, 6, new Color8Bit(255, 255, 255)));
  MechanismLigament2d upperArm = armUprights.append(new MechanismLigament2d("UpperArm", 
                                    Units.inchesToMeters(ArmConstants.ARM_LENGTH_INCHES),
                                    90.0 - ArmConstants.ANGLE_OFFSET_FROM_VERTICAL_DEGREES, 6, new Color8Bit(255, 255, 0)));
  MechanismLigament2d foreArm = upperArm.append(new MechanismLigament2d("Forearm",
                                    Units.inchesToMeters(23),
                                    180.0 - ArmConstants.ANGLE_OFFSET_FROM_ZERO, 6, new Color8Bit(255, 0, 255)));

  public Arm(Claw claw) {
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    encoder = ArmConstants.USE_LEFT_ENCODER ? leftMotor.getAbsoluteEncoder(Type.kDutyCycle) : rightMotor.getAbsoluteEncoder(Type.kDutyCycle); 
    encoder.setInverted(!ArmConstants.USE_LEFT_ENCODER);
    encoder.setPositionConversionFactor(360.0);
    encoder.setVelocityConversionFactor(encoder.getPositionConversionFactor());
    
    pidController = leftMotor.getPIDController();
    pidController.setFeedbackDevice(encoder);

    // Set PID values
    pidController.setP(ArmConstants.PID_kP, 0);
    pidController.setI(ArmConstants.PID_kI, 0);
    pidController.setD(ArmConstants.PID_kD, 0);
    pidController.setFF(ArmConstants.PID_kFF, 0);
    pidController.setOutputRange(-0.35, 0.35, 0);
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
    pidController.setReference(desiredAngle, ControlType.kPosition, 0);

    double armAbsAngle = ArmConstants.ANGLE_OFFSET_FROM_VERTICAL_DEGREES + encoder.getPosition() - ArmConstants.ANGLE_OFFSET_FROM_ZERO;
    upperArm.setAngle(armAbsAngle - 90.0);
    foreArm.setAngle(180.0 - armAbsAngle);

    SmartDashboard.putNumber("Arm Angle", encoder.getPosition());
    SmartDashboard.putNumber("Arm Velocity", encoder.getVelocity());
    SmartDashboard.putNumber("Set Point", desiredAngle);

    SmartDashboard.putNumber("Percent Range", pidController.getOutputMax(0));
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
   * Returns true if the claw is currently closed and false if it is currently open
   * Best used as a boolean supplier for a {@code waitUntil} command
   * @return Truth state of the statment "the claw is closed"
   */
  public boolean getHolding(){
    return claw.getClawState().equals(ClawState.Open);
  }

  /**
   * Checks whether or not the arm is currently at the desired angle within a given tolerance defined in the constants.
   * Best used as a boolean supplier with a {@code waitUntil} command
   * @return If the arm is at the desired angle
   */
  public boolean atSetPoint(){
    if (Math.abs(encoder.getPosition() - desiredAngle) <= ArmConstants.ANGLE_CHECKTOLERANCE_DEGREES){
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
    if (encoder.getPosition() <= 13.0){
      desiredAngle = 0.0;
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

  /**
   * Puts all PID values on SmartDashboard
   */
  public void putPIDValues(){
    SmartDashboard.putNumber("P", pidController.getP(0));
    SmartDashboard.putNumber("I", pidController.getI(0));
    SmartDashboard.putNumber("D", pidController.getD(0));
    SmartDashboard.putNumber("FF", pidController.getFF(0));
    SmartDashboard.putNumber("Min percent", pidController.getOutputMin(0));
    SmartDashboard.putNumber("Max percent", pidController.getOutputMax(0));
    SmartDashboard.putNumber("MaxAcceleration", pidController.getSmartMotionMaxAccel(0));
    SmartDashboard.putNumber("MaxVelocity", pidController.getSmartMotionMaxVelocity(0));
    SmartDashboard.putNumber("MinVelocity", pidController.getSmartMotionMinOutputVelocity(0));
    SmartDashboard.putNumber("MinError", pidController.getSmartMotionAllowedClosedLoopError(0));
  }

  /**
   * Sets all PID values to SmartDashboard values
   */
  public void reassignPIDValues(){
    pidController.setP(SmartDashboard.getNumber("P", ArmConstants.PID_kP), 0);
    pidController.setI(SmartDashboard.getNumber("I", ArmConstants.PID_kI), 0);
    pidController.setD(SmartDashboard.getNumber("D", ArmConstants.PID_kD), 0);
    pidController.setFF(SmartDashboard.getNumber("FF", ArmConstants.PID_kFF), 0);
    pidController.setOutputRange(SmartDashboard.getNumber("Min percent", -0.35), SmartDashboard.getNumber("Max percent", 0.35), 0);
    pidController.setSmartMotionMaxAccel(SmartDashboard.getNumber("MaxAcceleration", 0.0), 0);
    pidController.setSmartMotionMaxVelocity(SmartDashboard.getNumber("MaxVelocity", 0.0), 0);
    pidController.setSmartMotionMinOutputVelocity(SmartDashboard.getNumber("MinVelocity", 0.0), 0);
    pidController.setSmartMotionAllowedClosedLoopError(SmartDashboard.getNumber("MinError", 0.0), 0);
  }

  /**
   * Sets the idle mode on the arm motors to the given idle mode
   * @param idleMode The idle mode for the arm motors to change to
   */
  public void setBrakes(IdleMode idleMode){
    leftMotor.setIdleMode(idleMode);
    rightMotor.setIdleMode(idleMode);
  }
  
  /**
   * Checks whether or not the PID should be dampened. Returns true if so and false otherwise
   * @return Whether or not the PID should be dampened
   */
  public boolean shouldDampen(){
    double encoderVelocity = encoder.getVelocity();
    double checkDirection = -1.0 * (encoderVelocity / Math.abs(encoderVelocity));
    double rampAngle = desiredAngle + (checkDirection * ArmConstants.ANGLE_RAMPDISTANCE_DEGREES);
    
    double error = checkDirection * (rampAngle - encoder.getPosition());

    return error >= 0;
  }

  /**
   * Checks whether or not the PID is fully damped. Returns trus if so and false otherwise.
   * @return Whether or not the PID is fully dampened
   */
  public boolean fullDamped(){
    return pidController.getOutputMax() <= ArmConstants.SPEED_DAMPEN_PERCENTOUTPUT;
  }

  /**
   * Sets the PID's output range back to full
   */
  public void undoDampen(){
    pidController.setOutputRange(-0.35, 0.35, 0);
  }

  /**
   * Lowers the PID's output range to give the illusion of a smooth ramp down.
   * Should be called recursively while the PID should be dampened
   */
  public void rampDown(){
    double error = Math.abs(encoder.getPosition() - desiredAngle);

    double correctedError = ArmConstants.ANGLE_RAMPDISTANCE_DEGREES - error;
    double speedOffsetFromMax = (correctedError * ArmConstants.SPEED_RAMPDOWNRATE_PERCENTPERDEGREE) / 100.0;
    double newPercentRange = ArmConstants.SPEED_FULL_PERCENTOUTPUT - speedOffsetFromMax;

    pidController.setOutputRange(-newPercentRange, newPercentRange, 0);
  }
}