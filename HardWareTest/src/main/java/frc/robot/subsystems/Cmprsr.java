// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CompressorConstants;

public class Cmprsr extends SubsystemBase {
  Compressor compressor;
  public Cmprsr() {
    compressor = new Compressor(CompressorConstants.COMPRESSOR_ID, PneumaticsModuleType.REVPH);
  }

  public void Compressor_Active(){
    compressor.enableAnalog(CompressorConstants.COMPRESSOR_RANGE_MIN, CompressorConstants.COMPRESSOR_RANGE_MAX);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
