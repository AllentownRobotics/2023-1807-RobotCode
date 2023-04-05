// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.GlobalConstants;

public class Compress extends SubsystemBase {
  Compressor comp;
  
  static Compress instance = null;

  /**
   * Creates a new Compress. 
   * NOTE: This method should not be manually called. Instead,
   * use the singleton instance by calling the static method {@link Compress#getInstance()} 
   */
  public Compress() {
    comp = new Compressor(GlobalConstants.PNEUMATICS_ID, PneumaticsModuleType.REVPH);
  }

  /**
   * Gets the singleton instance of the compress. If no instance exists one is automatically created.
   * @return The singleton instance of the compress
   */
  public static Compress getInstance(){
    if (instance == null){
      instance = new Compress();
    }
    return instance;
  }

  public void run(){
    SmartDashboard.putNumber("Pressure", comp.getPressure());
    SmartDashboard.putBoolean("Compressor Running", comp.isEnabled());
    comp.enableAnalog(60,120);
  }
}
