// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Utils.Constants.DriveConstants;
import frc.robot.Utils.Constants.FieldConstants;

public class Limelight extends SubsystemBase {
  public NetworkTable table;

  public static double x;
  public static double y;
  public static double ta;
  public static double[] targetRelPos;
  public static double[] RoboRelPos;

  public static boolean tv;

  public double LimePipeline = 1;
  public double April2DPipeline = 0;
  public double April3DPipeline = 9;

  public double purplePipeline = 2;
  public double yellowPipeline = 3;
  public double HumanPlayerAprilTag = 5;
  public double RightShelfPOI = 8;

  public static double currentPipeline;

  RobotContainer rc;

  private SwerveDriveOdometry localTargetSpaceOdometry;

  /** Creates a new ExampleSubsystem. */
  public Limelight(RobotContainer rc) {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    currentPipeline = April2DPipeline;

    this.rc = rc;
  }

  public void initLocalOdometry(DriveTrain drive){
    localTargetSpaceOdometry = new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS, 
    drive.getHeading(),
    drive.getModulePositions());
  }

  /**
   * Example command factory method.
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

  public CommandBase setApril2DPipe() {
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

  public double[] april3DCordsBotPoseTargetSpace() {
    return table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
  }

  public boolean trackingApril() {
    if (table.getEntry("pipeline").getDouble(0.0) == 0) {
      return true;
    } else {return false;}

  }

  public Pose2d robotPoseTargetSpace(){
    double[] values = april3DCordsBotPoseTargetSpace();
    return new Pose2d(new Translation2d(values[0], values[1]), new Rotation2d(values[4]));
  }

  public void resetLocalOdometryPosition(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions){
    localTargetSpaceOdometry.resetPosition(gyroAngle, modulePositions, robotPoseTargetSpace());
  }

  public boolean targetAquired() {
    return tv;
  }

  public PathPlannerTrajectory generateNodeTrajectory(Rotation2d robotOrientation, double offset, double xVel, double yVel){
    Rotation2d heading = new Rotation2d(xVel, yVel);
    Translation2d targetNodePosition = new Translation2d(offset, -0.75);
    return PathPlanner.generatePath(
      new PathConstraints(3, 3),
      new PathPoint(localTargetSpaceOdometry.getPoseMeters().getTranslation(), heading, robotOrientation),
      new PathPoint(targetNodePosition, heading, new Rotation2d(Math.PI))
    );
  }

  public PathPlannerTrajectory generateSubstationTrajectory(Pose2d robotPose, double xVel, double yVel){
    Rotation2d heading = new Rotation2d(xVel, yVel);
    return PathPlanner.generatePath(
      new PathConstraints(3, 3),
      new PathPoint(robotPose.getTranslation(), heading, robotPose.getRotation()),
      new PathPoint(FieldConstants.SINGLESUBSTATION_ALLIANCERELATIVE, heading, new Rotation2d(Math.PI / 2.0))
    );
  }

  public Pose2d robotPoseAllianceSpace(){
    String entryName = DriverStation.getAlliance().equals(Alliance.Blue) ? "botpose_wpiblue" : "botpose_wpired";
    double[] values = table.getEntry(entryName).getDoubleArray(new double[6]);

    return new Pose2d(new Translation2d(values[0], values[1]), new Rotation2d(values[4]));
  }

  public SwerveDriveOdometry getLocalOdometryInstance(){
    return localTargetSpaceOdometry;
  }

  public boolean isTargetNodeTag(){
    if (!tv){
      return false;
    }

    double id = table.getEntry("tid").getDouble(0);
    if (id >= 8.0 && id <= 6.0){
      return true;
    }
    if (id <= 3.0 && id >= 1.0){
      return true;
    }

    return false;
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
    } catch (Exception e) {}

    localTargetSpaceOdometry.update(rc.drive.getHeading(), rc.drive.getModulePositions());
  }

}
