// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.ColorMatchType;

public class Spindexer extends SubsystemBase {
  CANSparkMax motor = new CANSparkMax(SpindexerConstants.SPINDEXER_MOTOR_ID, MotorType.kBrushless);

  ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  public Spindexer() {
    motor.setIdleMode(IdleMode.kBrake);

    motor.burnFlash();
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Color Sensor", colorMatch(colorSensor.getColor()).asString);
  }

  public void spindex(double direction){
    motor.set(direction * SpindexerConstants.SPINDEXER_MOTOR_MAXOUTPUT);
  }

  public ColorMatchType colorMatch(Color sensedColor){
    double sRed = sensedColor.red;
    double sGreen = sensedColor.green;
    double sBlue = sensedColor.blue;

    double cubeThreshold = SpindexerConstants.COLORSENSOR_THRESHOLD_CUBE;
    double coneThreshold = SpindexerConstants.COLORSENSOR_THRESHOLD_CONE;

    Color cubeColor = SpindexerConstants.COLORSENSOR_COLOR_CUBE;
    Color coneColor = SpindexerConstants.COLORSENSOR_COLOR_CONE;

    // Plot RGB as XYZ point for sensed color and for constant color, find 3D distance (deltaE) to compare against threshold
    // Produces sphere around constant colors that sensed colors must fall wihtin
    // Tighter and more accurate range compared to previous which created cube around color constants
    double cubeDeltaE = Math.sqrt(Math.pow(sRed - cubeColor.red, 2.0) + Math.pow(sGreen - cubeColor.green, 2) + Math.pow(sBlue, cubeColor.blue));
    double coneDeltaE = Math.sqrt(Math.pow(sRed - coneColor.red, 2.0) + Math.pow(sGreen - coneColor.green, 2) + Math.pow(sBlue, coneColor.blue));

    // Dual guard clause
    if (coneDeltaE < coneThreshold){
      return ColorMatchType.ConeMatch;
    }
    if (cubeDeltaE < cubeThreshold){
      return ColorMatchType.CubeMatch;
    }

    return ColorMatchType.NullMatch;
  }
}
