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

public class Arm extends SubsystemBase {
  CANSparkMax leftMotor = new CANSparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);

  CANSparkMax rightMotor = new CANSparkMax(ArmConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

  AbsoluteEncoder encoder;

  SparkMaxPIDController pidController;
  FauxTrapezoidProfile profile = new FauxTrapezoidProfile(210.0, 3.0 * 1.4, 1.0, 2.5);

  double desiredAngle;
  double manualSpeed;
  boolean automaticControl;

  static Arm instance;

  /**
   * Creates a new Arm. 
   * NOTE: This method should not be manually called. Instead,
   * use the singleton instance by calling the static method {@link Arm#getInstance()} 
   */
  public Arm() {
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    encoder = ArmConstants.USE_LEFT_ENCODER ? leftMotor.getAbsoluteEncoder(Type.kDutyCycle) : rightMotor.getAbsoluteEncoder(Type.kDutyCycle); 
    encoder.setInverted(!ArmConstants.USE_LEFT_ENCODER);
    encoder.setPositionConversionFactor(360.0);
    encoder.setVelocityConversionFactor(360.0 / 60.0);

    leftMotor.setInverted(false);

    rightMotor.follow(leftMotor, true);

    pidController = leftMotor.getPIDController();
    pidController.setFeedbackDevice(encoder);

    pidController.setP(ArmConstants.PID_kP, 0);
    pidController.setI(ArmConstants.PID_kI, 0);
    pidController.setD(ArmConstants.PID_kD, 0);
    pidController.setFF(ArmConstants.PID_kFF, 0);
    pidController.setOutputRange(-0.75, 0.75, 0);

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    leftMotor.setSmartCurrentLimit(40);
    rightMotor.setSmartCurrentLimit(40);

    leftMotor.burnFlash();
    rightMotor.burnFlash();

    desiredAngle = 0.0;

    profile.setGoal(0.0);

    manualSpeed = 0.0;

    automaticControl = true;
  }

  /**
   * Gets the singleton instance of the arm. If no instance exists one is automatically created.
   * @return The singleton instance of the arm
   */
  public static Arm getInstance(){
    if (instance == null){
      instance = new Arm();
    }
    return instance;
  }

  @Override
  public void periodic() {
    double profileOutput = profile.calculate(encoder.getPosition());
    pidController.setReference(automaticControl ? profileOutput : manualSpeed, ControlType.kVelocity, 0);

    SmartDashboard.putNumber("Arm Angle", encoder.getPosition());
    SmartDashboard.putNumber("Set Point", desiredAngle);
    SmartDashboard.putBoolean("At Set Point", atSetPoint());
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

  /**
   * Checks if the arm is currently at the desired setpoint and returns accordingly
   * @return Whether or not the arm is at the setpoint
   */
  public boolean atSetPoint(){
    return Math.abs(getPositionalError()) <= ArmConstants.ANGLE_CHECKTOLERANCE_DEGREES;
  }

  /**
   * The current angle of the arm in degrees
   * @return Current angle of the arm in degrees
   */
  public double getArmAngle(){
    return encoder.getPosition();
  }

  /**
   * Checks if the arm is within the wrist exclusion zone and returns accordingly
   * @return Whether or not the arm is in the exclusion zone
   */
  public boolean isWristAllowedOut(){
    return !(encoder.getPosition() >= ClawConstants.ANGLE_WRIST_EXCLUSIONZONE_MIN && encoder.getPosition() <= ClawConstants.ANGLE_WRIST_EXCLUSIONZONE_MAX);
  }

  /**
   * Open loop control of the motors on the arm
   * @param percentOutput Percent for the motors to run at
   */
  public void openLoopMotorControl(double percentOutput){
    leftMotor.set(percentOutput);
  }

  /**
   * Sets the brake idle mode of the arm motors
   * @param mode Idle mode for the motors to change to
   */
  public void setBrakes(IdleMode mode){
    leftMotor.setIdleMode(mode);
    rightMotor.setIdleMode(mode);
  }

  /**
   * Sets the manual speed of the arm
   * @param angularSpeed The speed for the arm to move at
   */
  public void setManualSpeed(double angularSpeed){
    manualSpeed = angularSpeed;
  }

  /**
   * Sets whether or not the arm should obey automatic control
   * @param useAuto Whether or not the arm should obey automatic control
   */
  public void setAutomaticMode(boolean useAuto){
    automaticControl = useAuto;
  }
}