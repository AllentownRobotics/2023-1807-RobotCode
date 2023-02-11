// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;


public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax leftArmMotor;
  private CANSparkMax rightArmMotor;

  private AbsoluteEncoder leftEncoder;
  private AbsoluteEncoder rightEncoder;

  private SparkMaxPIDController leftArmPIDController;

  public Arm() {
    leftArmMotor = new CANSparkMax(ArmConstants.armMotorLeftCanId, MotorType.kBrushless);
    rightArmMotor = new CANSparkMax(ArmConstants.armMotorRightCanId, MotorType.kBrushless);

    leftEncoder = leftArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
    rightEncoder = rightArmMotor.getAbsoluteEncoder(Type.kDutyCycle);

    leftArmPIDController = leftArmMotor.getPIDController();

    leftArmPIDController.setFeedbackDevice(leftEncoder);

    leftArmPIDController.setP(ArmConstants.armP);
    leftArmPIDController.setI(ArmConstants.armI);
    leftArmPIDController.setD(ArmConstants.armD);
    leftArmPIDController.setFF(ArmConstants.armFF);
    leftArmPIDController.setOutputRange(ArmConstants.armMinOutput,
        ArmConstants.armMaxOutput);
  }

  @Override
  public void periodic() {
    if(rightEncoder.getPosition()!=leftEncoder.getPosition())
    {
      SmartDashboard.putString("ArmEncoderReset", "ENCODERS NOT ALIGNED");
    }
    else
    {
      SmartDashboard.putString("ArmEncoderReset", "Encoders Aligned :)");

    }
    if(RobotContainer.cubeMode)
    {
      SmartDashboard.putString("Mode", "Cube Mode");
    }
    else
    {
      SmartDashboard.putString("Mode", "Cube Mode");
    }
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void set(double angle)
  {
    leftArmPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }
}
