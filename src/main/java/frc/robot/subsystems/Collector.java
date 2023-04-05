// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Enums.ClawState;
import frc.robot.Utils.Enums.WristState;

/**
 * This subsystem uses seemingly unrelated enum classes for interaction and state swtiching
 */
public class Collector extends SubsystemBase {
  CANSparkMax leftRoller = new CANSparkMax(21, MotorType.kBrushless);
  CANSparkMax rightRoller = new CANSparkMax(22, MotorType.kBrushless);

  CANSparkMax wristMotor = new CANSparkMax(20, MotorType.kBrushless);
  AbsoluteEncoder wristEncoder;
  SparkMaxPIDController wristController;

  Rev2mDistanceSensor sensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kMillimeters, RangeProfile.kDefault);
  //TimeOfFlight sensor = new TimeOfFlight(19);

  DoubleSolenoid piston = new DoubleSolenoid(14, PneumaticsModuleType.REVPH, 2, 6);

  ClawState desiredGripState = ClawState.Closed;
  WristState desiredFlipState = WristState.WristIn;

  boolean autoGrabEnabled = false;

  private static Collector instance;

  /**
   * Creates a new Collector. 
   * <p>NOTE: This method should not be manually called. Instead,
   * use the singleton instance by calling the static method {@link Collector#getInstance()} 
   */
  public Collector() {
    leftRoller.restoreFactoryDefaults();
    rightRoller.restoreFactoryDefaults();
    wristMotor.restoreFactoryDefaults();

    wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    wristEncoder.setPositionConversionFactor(360.0);
    wristEncoder.setVelocityConversionFactor(360.0);
    
    rightRoller.follow(leftRoller, true);

    wristController = wristMotor.getPIDController();
    wristController.setFeedbackDevice(wristEncoder);
    wristController.setP(0.02);
    wristController.setI(0.0);
    wristController.setD(0.0);
    wristController.setFF(0.0);
    wristController.setOutputRange(-0.3, 0.3);

    leftRoller.setIdleMode(IdleMode.kBrake);
    rightRoller.setIdleMode(IdleMode.kBrake);
    wristMotor.setIdleMode(IdleMode.kBrake);

    leftRoller.burnFlash();
    rightRoller.burnFlash();
    wristMotor.burnFlash();

    sensor.setEnabled(true);
    sensor.setAutomaticMode(true);
    sensor.setMeasurementPeriod(0.02);
  }

  /**
   * Gets the singleton instance of the collector. If no instance exists one is automatically created.
   * @return The singleton instance of the collector
   */
  public static Collector getInstance(){
    if (instance == null){
      instance = new Collector();
    }
    return instance;
  }

  public void setFlipState(WristState desiredState){
    desiredFlipState = desiredState;
  }

  /**
   * Sets the currnet grip state of the collector
   * @param desiredState The desired grip state
   */
  public void setGripState(ClawState desiredState){
    desiredGripState = desiredState;
  }

  /**
   * Checks if there is a game piece in range to be grabbed
   * @return Whether or not a game piece is in range 
   */
  public boolean pieceInRange(){
    return sensor.GetRange() <= 350.0 && sensor.isRangeValid() && wristEncoder.getPosition() >= 50.0;
  }

  /**
   * Sets usage of automatic grabbing
   * @param enabled Whether or not usage should be allowed
   */
  public void setAutoGrabUsage(boolean enabled){
    autoGrabEnabled = enabled;
  }

  /**
   * The current usage of automatic grabbing
   * @return Whether or not automatic grabbing is enabled
   */
  public boolean isAutoGrabEnabled(){
    return autoGrabEnabled;
  }

  /**
   * Runs the wheels at a maximum 75% speed to spit the currently held game piece in the given direction.
   * 
   * <p> {@code -1} for into the bot, {@code 1} for out of the bot
   * @param direction The direction to spit the piece in
   */
  public void spit(double direction){
    leftRoller.set(direction * 0.75);
  }

  /**
   * Toggles the current grip state between closed and open
   */
  public void toggleGripState(){
    if (desiredGripState.equals(ClawState.Open)){
      desiredGripState = ClawState.Closed;
      return;
    }
    desiredGripState = ClawState.Open;
  }

  /**
   * Gets the current angle of the collector wrist in degrees
   * @return The angle of the collector in degrees
   */
  public double getCurrentWristAngle(){
    return wristEncoder.getPosition();
  }

  @Override
  public void periodic() {
    piston.set(desiredGripState.equals(ClawState.Open) ? Value.kForward : Value.kReverse);
    wristController.setReference(desiredFlipState.equals(WristState.WristOut) ? 100.0 : 10.0, ControlType.kPosition);

    SmartDashboard.putNumber("Distance", sensor.getRange());
    SmartDashboard.putBoolean("Open", desiredGripState.equals(ClawState.Open));
    SmartDashboard.putBoolean("Piece in range", pieceInRange());
  }
}
