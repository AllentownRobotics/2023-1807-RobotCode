// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  public NetworkTable table;

  public static double x;
  public static double y;
  public static double[] targetRelPos;
  public static double[] RoboRelPos;

  public static boolean tv;

  public double LimePipeline = 1;
  public double AprilPipeline = 0;
  public double purplePipeline = 2;
  public double yellowPipeline = 3;
  public double HumanPlayerAprilTag = 5;
  public double RightShelfPOI = 8;




  public static double currentPipeline;

  /** Creates a new ExampleSubsystem. */
  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    currentPipeline = AprilPipeline;
  }

  /**
   * Example command factory method.
   * @return 
   *
   * @return a command
   */
  public CommandBase LightOn() {
    return runOnce(
        () -> {
          table.getEntry("ledMode").setDouble(3);
        });
  }

  public CommandBase LightOff() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          table.getEntry("ledMode").setDouble(1);
        });
  }


  public CommandBase setLimePipe() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          table.getEntry("pipeline").setDouble(LimePipeline);
          currentPipeline = LimePipeline;
        });
  }

  public CommandBase setAprilPipe() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          table.getEntry("pipeline").setDouble(AprilPipeline);
          currentPipeline = AprilPipeline;
        });
  }

  public CommandBase setConePipe() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          table.getEntry("pipeline").setDouble(yellowPipeline);
          currentPipeline = yellowPipeline;
        });
  }

  public CommandBase setCubePipe() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          table.getEntry("pipeline").setDouble(purplePipeline);
          currentPipeline = purplePipeline;
        });
  }

  public CommandBase setHPpipe() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          table.getEntry("pipeline").setDouble(HumanPlayerAprilTag);
          currentPipeline = HumanPlayerAprilTag;
        });
  }

  public CommandBase setShelfPipe() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          table.getEntry("pipeline").setDouble(RightShelfPOI);
          currentPipeline = RightShelfPOI;
        });
  }

  public CommandBase TapeTracking() {
    return Commands.sequence(LightOn(), setLimePipe());
  }

  public CommandBase AprilTracking() {
    return Commands.sequence(LightOff(), setAprilPipe());
  }

  public CommandBase ConeTracking() {
    return Commands.sequence(LightOff(), setConePipe());
  }  
  
  public CommandBase CubeTracking() {
    return Commands.sequence(LightOff(), setCubePipe());
  }

  public CommandBase HPStationTracking() {
    return Commands.sequence(LightOff(), setHPpipe());
  }

  public CommandBase ShelfTracking() {
    return Commands.sequence(LightOff(), setShelfPipe());
  }

  public double angleX() {
    return x;
  }

  public double angleY() {
    return y;
  }


  public boolean targetAquired() {
    return tv;
  }


  public static double distanceFromTargetMeters() {
    
    if (currentPipeline == 0 || currentPipeline == 5 || currentPipeline ==8) {
      return targetRelPos[2];
    } else if (currentPipeline == 1 || currentPipeline == 3) {
      double LLHeight = 0;
      double LLAngle = 0;
      double TargetHeight = 0;
      double combinedAngle = LLAngle - y;
      return (TargetHeight - LLHeight)*Math.tan(Units.degreesToRadians(combinedAngle));
    }

     else {return 0;}
  }

  public double[] april3DCords() {
    return table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
  }

  public boolean trackingApril() {
    if (table.getEntry("pipeline").getDouble(0.0) == 0) {
      return true;
    } else {return false;}

  }


  

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    RoboRelPos = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    targetRelPos = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);

    x = table.getEntry("tx").getDouble(0.0);
    y = table.getEntry("ty").getDouble(0.0);

    if (table.getEntry("tv").getDouble(0.0) == 1) {
      tv = true;
    } else {tv = false;}

    SmartDashboard.putNumber("X", x);
    SmartDashboard.putNumber("Y", y);

    SmartDashboard.putBoolean("Target Aquired", tv);
 
    SmartDashboard.putNumber("Target Spec X", RoboRelPos[0]);
    SmartDashboard.putNumber("Target Spec Y", RoboRelPos[1]);
    SmartDashboard.putNumber("Target Spec Z", RoboRelPos[2]);
    SmartDashboard.putNumber("Target Spec Roll", RoboRelPos[3]);
    SmartDashboard.putNumber("Target Spec Pitch", RoboRelPos[4]);
    SmartDashboard.putNumber("Target Spec Yaw", RoboRelPos[5]);

    SmartDashboard.putNumber("Pipeline", currentPipeline);





  }

}
