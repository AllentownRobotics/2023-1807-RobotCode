// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  public static NetworkTable table;

  public static double x;
  public static double y;
  public static double ta;
  public static double[] targetRelPos;
  public static double[] RoboRelPos;

  public static boolean tv;

  public static double LimePipeline = 1;
  public static double April2DPipeline = 0;
  public static double April3DPipeline = 9;

  public static double purplePipeline = 2;
  public static double yellowPipeline = 3;
  public static double HumanPlayerAprilTag = 5;
  public static double RightShelfPOI = 8;

  public static double currentPipeline;

  static Limelight instance = null;

  /**
   * Creates a new Limelight. 
   * NOTE: This method should not be manually called. Instead,
   * use the singleton instance by calling the static method {@link Limelight#getInstance()} 
   */
  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    currentPipeline = April2DPipeline;

    table.getEntry("pipeline").setDouble(April2DPipeline);
    currentPipeline = April2DPipeline;
  }

  /**
   * Gets the singleton instance of the limelight. If no instance exists one is automatically created.
   * @return The singleton instance of the limelight
   */
  public static Limelight getInstance(){
    if (instance == null){
      instance = new Limelight();
    }
    return instance;
  }

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

  public CommandBase  setApril2DPipe() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          table.getEntry("pipeline").setDouble(April2DPipeline);
          currentPipeline = April2DPipeline;
        });
  }

  public CommandBase setApril3DPipe() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          table.getEntry("pipeline").setDouble(April2DPipeline);
          currentPipeline = April3DPipeline;
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

  public CommandBase April2DTracking() {
    return Commands.sequence(LightOff(), setApril2DPipe());
  }

  public double angleX() {
    return x;
  }

  public double angleY() {
    return y;
  }

  public static double distanceFromTargetMeters() {
    
    if (currentPipeline == 0 || currentPipeline == 5 || currentPipeline ==8) {
      return targetRelPos[2];
    } else if (currentPipeline == 1 || currentPipeline == 3) {
      double LLHeight = 0;
      double TargetHeight = 0;
      double combinedAngle = TargetHeight - y;
      return (TargetHeight - LLHeight) * Math.tan(Units.degreesToRadians(combinedAngle));
    }

     else {return 0;}
  }

  public static double[] april3DCordsBotPoseTargetSpace() {
    return table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
  }

  public static double[] targetPoseRobotSpace(){
    return table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
  }

  public boolean trackingApril() {
    if (table.getEntry("pipeline").getDouble(0.0) == 0) {
      return true;
    } else {return false;}

  }

  /**
   * Gets the robot's pose relative to the in-view apriltag
   * @return The robot's pose relative to the in-view apriltag
   */
  public Pose2d robotPoseTargetSpace(){
    double[] values = april3DCordsBotPoseTargetSpace();
    try{
      return new Pose2d(new Translation2d(values[0], values[2]), new Rotation2d(values[4]));
    }
    catch (Exception e){
      return new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
    }
  }

  /**
   * Checks whether the limelight has a valid target in view. Returns true if so and false otherwise
   * @return Whether or not the limelight has a valid target in view
   */
  public boolean targetAquired() {
    return tv;
  }

  /**
   * Gets the robot's pose relative to the robot's alliance.
   * @return The robot's pose relative to the robot's alliance
   */
  public Pose2d robotPoseAllianceSpace(){
    String entryName = DriverStation.getAlliance().equals(Alliance.Blue) ? "botpose_wpiblue" : "botpose_wpired";
    double[] values = table.getEntry(entryName).getDoubleArray(new double[6]);

    return new Pose2d(new Translation2d(values[0], values[2]), new Rotation2d(values[4]));
  }

  /**
   * Checks whether or not the current in-view apriltag is a node tag. Returns true if so and false otherwise
   * @return Whether the in-view apriltag is a node tag
   */
  public static boolean isTargetNodeTag(){
    if (!tv){
      return false;
    }

    double id = table.getEntry("tid").getDouble(0);
    return (id <= 8.0 && id >= 6.0) || (id <= 3.0 && id >= 1.0);
  }

  public void setPipeline(double pipeline){
    table.getEntry("pipeline").setDouble(pipeline);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    try {
      RoboRelPos = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

      targetRelPos = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);

      

      x = table.getEntry("tx").getDouble(0.0);
      y = table.getEntry("ty").getDouble(0.0);
      ta = table.getEntry("ta").getDouble(0.0);


      if (table.getEntry("tv").getDouble(0.0) == 1) {
        tv = true;
      } else {tv = false;}

    } catch (Exception e) {}
  }

}
