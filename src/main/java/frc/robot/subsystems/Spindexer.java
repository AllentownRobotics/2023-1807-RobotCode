// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {
  CANSparkMax motor = new CANSparkMax(SpindexerConstants.SPINDEXER_MOTOR_ID, MotorType.kBrushless);
  CANSparkMax roller = new CANSparkMax(33, MotorType.kBrushless);
  
  static Spindexer instance = null;

  public Spindexer() {
    motor.setIdleMode(IdleMode.kBrake);

    roller.restoreFactoryDefaults();

    roller.setInverted(true);
  }

  public static Spindexer getInstance(){
    if (instance == null){
      instance = new Spindexer();
    }
    return instance;
  }

  @Override
  public void periodic() {}

  /**
   * Runs the motor at the max allowed speed with the direction
   * @param direction the direction for the spindexer to spin 
   */
  public void spindex(double direction){
    motor.set(direction * SpindexerConstants.SPINDEXER_MOTOR_MAXOUTPUT);
    roller.set(Math.abs(direction) * SpindexerConstants.SPINDEXER_MOTOR_MAXOUTPUT);
  }

  public void runRoller(double direction){
    roller.set(direction * SpindexerConstants.SPINDEXER_MOTOR_MAXOUTPUT);
  }
}
 
