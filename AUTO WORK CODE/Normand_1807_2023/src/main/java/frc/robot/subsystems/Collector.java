// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.CompressorConstants;

public class Collector extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  CANSparkMax leftCollectMotor;
  CANSparkMax rightCollectMotor;
  DoubleSolenoid collectSolenoid;

  public Collector() {
    leftCollectMotor = new CANSparkMax(CollectorConstants.collectMotorLeftCanId, MotorType.kBrushless);
    rightCollectMotor = new CANSparkMax(CollectorConstants.collectMotorRightCanId, MotorType.kBrushless);
    leftCollectMotor.setIdleMode(IdleMode.kBrake);
    rightCollectMotor.setIdleMode(IdleMode.kBrake);
    collectSolenoid = new DoubleSolenoid(CompressorConstants.compressorCanId,PneumaticsModuleType.REVPH, CollectorConstants.collectForwardChannel, CollectorConstants.collectReverseChannel);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void collect(double speed)
  {
    collectSolenoid.set(Value.kForward);
    leftCollectMotor.set(speed);
    rightCollectMotor.set(-speed);
  }
  public void retract()
  {
    collectSolenoid.set(Value.kForward);
    leftCollectMotor.set(0);
    rightCollectMotor.set(0);
  }
}

