// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
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

  double disiredangle;

  public Arm() {
    leftArmMotor = new CANSparkMax(ArmConstants.armMotorLeftCanId, MotorType.kBrushless);
    rightArmMotor = new CANSparkMax(ArmConstants.armMotorRightCanId, MotorType.kBrushless);
    
    leftArmMotor.restoreFactoryDefaults();
    rightArmMotor.restoreFactoryDefaults();
    
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);

    leftEncoder = leftArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
    rightEncoder = rightArmMotor.getAbsoluteEncoder(Type.kDutyCycle);

    leftEncoder.setPositionConversionFactor(360);
    leftEncoder.setVelocityConversionFactor(leftEncoder.getPositionConversionFactor()/60);
    leftEncoder.setInverted(false);

    leftArmPIDController = leftArmMotor.getPIDController();

    leftArmPIDController.setFeedbackDevice(leftEncoder);

    leftArmPIDController.setP(ArmConstants.armP);
    leftArmPIDController.setI(ArmConstants.armI);
    leftArmPIDController.setD(ArmConstants.armD);
    leftArmPIDController.setFF(ArmConstants.armFF);
    leftArmPIDController.setOutputRange(ArmConstants.armMinOutput,
        ArmConstants.armMaxOutput);
    leftArmPIDController.setPositionPIDWrappingEnabled(false);

    leftArmMotor.setInverted(false);
    rightArmMotor.follow(leftArmMotor, true);

    leftArmMotor.setSmartCurrentLimit(40);
    rightArmMotor.setSmartCurrentLimit(40);

    leftArmMotor.burnFlash();
    rightArmMotor.burnFlash();

    disiredangle = 0.0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle",leftEncoder.getPosition());
    SmartDashboard.putBoolean("ArmEncodersAligned", rightEncoder.getPosition()==leftEncoder.getPosition());
    if(RobotContainer.cubeMode){
      SmartDashboard.putString("Mode", "Cube Mode");}
    else{
      SmartDashboard.putString("Mode", "Cube Mode");}
    // This method will be called once per scheduler run

    leftArmPIDController.setReference(disiredangle, ControlType.kPosition);
    SmartDashboard.putNumber("desiredangle",disiredangle);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void set(double angle)
  {
    disiredangle = angle;
  }

  public double getAngle()
  {
    return leftEncoder.getPosition();
  }

  public double getDesired()
  {
    return disiredangle;
  }
}
