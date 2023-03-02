// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants.DriveConstants;
import frc.robot.Utils.Constants.GlobalConstants;

public class DriveTrain extends SubsystemBase {
 
 //gyro
 private final WPI_Pigeon2 gyro = new WPI_Pigeon2(GlobalConstants.PIGEON_ID);
 
  //creates all swerve modules 
private SwerveModule FL = new SwerveModule(
  DriveConstants.FL_DRIVE_ID,
  DriveConstants.FL_TURN_ID,
  DriveConstants.FL_CHASSIS_OFFSET
);

private SwerveModule FR = new SwerveModule(
  DriveConstants.FR_DRIVE_ID,
  DriveConstants.FR_TURN_ID,
  DriveConstants.FR_CHASSIS_OFFSET
);

private SwerveModule BL = new SwerveModule(
  DriveConstants.BL_DRIVE_ID,
  DriveConstants.BL_TURN_ID,
  DriveConstants.BL_CHASSIS_OFFSET
);
private SwerveModule BR = new SwerveModule(
  DriveConstants.BR_DRIVE_ID,
  DriveConstants.BR_TURN_ID,
  DriveConstants.BR_CHASSIS_OFFSET
);



//odometry class to track robot pose 
SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS, 
    gyro.getRotation2d(), 
   new SwerveModulePosition[]{
      FL.getPosition(),
     FR.getPosition(),
                      BL.getPosition(),
                      BR.getPosition()
                    });


public DriveTrain() {}

@Override
public void periodic() {
  // update odometry
  odometry.update(gyro.getRotation2d(), 
  new SwerveModulePosition[]{
    FL.getPosition(),
    FR.getPosition(),
    BL.getPosition(),
    BR.getPosition()
});
}


//returns current estimated pose of the robot
public Pose2d getPose(){
return odometry.getPoseMeters();
}

//resets odometry to a specific pose 
public void resetOdometry(Pose2d pose2d){
  odometry.resetPosition
    (gyro.getRotation2d(), 
    new SwerveModulePosition[]{
      FL.getPosition(),
      FR.getPosition(),
      BL.getPosition(),
      BR.getPosition()
      }, pose2d);
}

/*drive method  w/ joystick info:
* xSpeed - speed in x direction (forwards) 
* ySpeed - speed in y direction (sideways)
* rot - angular rate 
* fieldRelative - whether or not x and y are relative to the field
*/
public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative){
xSpeed *= DriveConstants.MAX_SPEED_MPS;
ySpeed *= DriveConstants.MAX_SPEED_MPS; 
rot *= DriveConstants.MAX_ANGLE_SPEED;

var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(fieldRelative
  ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
  : new ChassisSpeeds(xSpeed, ySpeed, rot));
SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SPEED_MPS);
  FL.setDesiredState(swerveModuleStates[0]);
  FR.setDesiredState(swerveModuleStates[1]);
  BL.setDesiredState(swerveModuleStates[2]);
  BR.setDesiredState(swerveModuleStates[3]);
  if(fieldRelative){
    SmartDashboard.putString("Orientation", "Field Orieanted");
  }else{                                                                       
    SmartDashboard.putString("Orientation", "Robot Oriented"); 
  }
}
//x lock 
public void setX(){
FL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
FR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
BL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
BR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
}

public void setModuleStates(SwerveModuleState[] desiredStates) {
  SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, DriveConstants.MAX_SPEED_MPS);
FL.setDesiredState(desiredStates[0]);
FR.setDesiredState(desiredStates[1]);
BL.setDesiredState(desiredStates[2]);
BR.setDesiredState(desiredStates[3]);
}

/** Resets the drive encoders to currently read a position of 0. */
public void resetEncoders() {
  FL.resetEncoders();
  BL.resetEncoders();
  FR.resetEncoders();
  BR.resetEncoders();
}
  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }


  //IN COMES SPRINGER STUFF
  public void setGyroDegrees(double yaw) {
    gyro.setYaw(yaw);
  }

  public CommandBase gyroSanityCheck() {
    return runOnce(
        () -> {
          gyro.setYaw(Limelight.targetRelPos[5] + 180);
        });
  }

  public CommandBase gyroSanityCheckHP() {
    return runOnce(
        () -> {
          gyro.setYaw(Limelight.targetRelPos[5]);
        });
  }

  public void levelSet(double speed) {
    FL.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
    FR.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
    BL.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
    BR.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
  }


  public double getPitch() {
    return gyro.getPitch();
  }

  public double getYaw() {
    return gyro.getYaw();
  }

  public double getRoll() {
    return gyro.getRoll();
  }

}
