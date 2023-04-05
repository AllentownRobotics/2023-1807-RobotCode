// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.SwerveUtils;
import frc.robot.Utils.Constants.DriveConstants;
import frc.robot.Utils.Constants.GlobalConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  // Create MAXSwerveModules

  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.FL_DRIVE_ID,
      DriveConstants.FL_TURN_ID,
      DriveConstants.FL_CHASSIS_OFFSET);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.FR_DRIVE_ID,
      DriveConstants.FR_TURN_ID,
      DriveConstants.FR_CHASSIS_OFFSET);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.BL_DRIVE_ID,
      DriveConstants.BL_TURN_ID,
      DriveConstants.BL_CHASSIS_OFFSET);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.BR_DRIVE_ID,
      DriveConstants.BR_TURN_ID,
      DriveConstants.BR_CHASSIS_OFFSET);

  // The gyro sensor
  private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(GlobalConstants.PIGEON_ID);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.DRIVE_KINEMATICS,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  private static DriveTrain instance = null;
  /** Creates a new DriveSubsystem. */
  public DriveTrain() {}

  public static DriveTrain getInstance(){
    if (instance == null){
      instance = new DriveTrain();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    SmartDashboard.putNumber("turn rate", getTurnRate());
    SmartDashboard.putNumber("moduleSpeed", m_frontLeft.getWheelVelocity());
    SmartDashboard.putNumber("desired", m_frontLeft.getDesiredState().speedMetersPerSecond);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.MAX_SPEED_MPS;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.MAX_SPEED_MPS;
    double rotDelivered = m_currentRotation * DriveConstants.MAX_ANGLE_SPEED;

    var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED_MPS);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * @param translateSpeed forward
   * @param strafeSpeed side to side
   * @param angularSpeed rotation
   */
  public void driveFromComponentSpeeds(double translateSpeed, double strafeSpeed, double angularSpeed){
    var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(translateSpeed, strafeSpeed, angularSpeed, m_gyro.getRotation2d()));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED_MPS);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }


  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }
  
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED_MPS);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Sets the desired states for the modules without optimizing
   * 
   * @param desiredStates The desired SwerveModule states
   */
  public void setModuleStatesNoOptimize(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED_MPS);

    m_frontLeft.setDesiredStateNoOpt(desiredStates[0]);
    m_frontRight.setDesiredStateNoOpt(desiredStates[1]);
    m_rearLeft.setDesiredStateNoOpt(desiredStates[2]);
    m_rearRight.setDesiredStateNoOpt(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Zeroes the heading of the robot. 
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Gets the current position of all the swerve modules
   * @return The current position of all the swerve modules
   */
  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
  }

  /**
   * Gets the current state of all the swerve modules
   * @return The current state of all the swerve modules
   */
  public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[]{
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };
  }

  /**
   * Returns the heading of the robot.
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeadingDegrees() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Gets the current heading of the robot
   * @ The robot's heading
   */
  public Rotation2d getHeading(){
    return m_gyro.getRotation2d();
  }

  /**
   * Gets the rotation of the robot on the roll axis
   * @return The roll of the robot
   */
  public double getRoll() {
    return m_gyro.getRoll();
  }

  /**
   * Drives at the given speed in forwards and backwards direction. Primarily used for leveling during autonomous
   * @param speed The speed to drive at
   */
  public void levelSet(double speed) {
    m_frontLeft.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
    m_frontRight.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
    m_rearLeft.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
    m_rearRight.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
  }
  
  /**
   * Gets the current chassis speeds of the robot for use to break down into component velocities
   * @return The current chassis speeds of the robot
   */
  public ChassisSpeeds getCompononetVelocities(){
    return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
  }

  public double getTilt(){
      return (Math.sqrt((Math.pow(m_gyro.getRoll(), 2)) + Math.pow(m_gyro.getPitch(), 2)));
  }
}