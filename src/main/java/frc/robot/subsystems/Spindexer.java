// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {


  private CANSparkMax spindexMotor;
  /** Creates a new ExampleSubsystem. */
  public Spindexer() {
    spindexMotor = new CANSparkMax(SpindexerConstants.SPINDEXER_MOTOR_ID, MotorType.kBrushless);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void spin(double speed)
  {
    spindexMotor.set(speed);
  }
}
 