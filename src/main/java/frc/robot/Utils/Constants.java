// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants{ 
    public static final int DRIVE_CONTROLLER = 1;
    public static final int OP_CONTROLLER = 0; 
    public static final double OP_CONTROLLER_THRESHOLD_SPINDEXER = 0.08;
  }

  public static class GlobalConstants{
    //Pneumatics
    public static final int PNEUMATICS_ID = 14;

    //Gyro ID
    public static final int PIGEON_ID = 9;
  }

  //DriveTrain
  public static final class DriveConstants{
    //Drive parameters 
    public static final double MAX_SPEED_MPS = 4.65;
    public static final double MAX_ANGLE_SPEED = 2 * Math.PI; //Radians per sec

    public static final double kDirectionSlewRate = 1.2 * 3.5; // radians per second
    public static final double kMagnitudeSlewRate = 1.8 * 3.5; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0 * 3.5; // percent per second (1 = 100%)

    //chassis config
    public static final double TRACK_WIDTH = Units.inchesToMeters(26.5); 
    public static final double WHEEL_BASE = Units.inchesToMeters(26.5);

    /**
     * Positions of each swerve module.
     * Follows global order ()
     */
    public static final Translation2d[] MODULE_POSITIONS = {
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)};

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(MODULE_POSITIONS);

    //angular offsets of modules from chassis
    public static final double FL_CHASSIS_OFFSET = -Math.PI / 2;
    public static final double FR_CHASSIS_OFFSET = 0.0; 
    public static final double BL_CHASSIS_OFFSET = Math.PI; 
    public static final double BR_CHASSIS_OFFSET = Math.PI / 2;

    //SPARK CAN IDS
    /*driving motor ids */
    /**Drive motor ID for the Front-Left module*/
    public static final int FL_DRIVE_ID = 1;
    /**Drive motor ID for the Back-Left module*/
    public static final int BL_DRIVE_ID = 5;
    /**Drive motor ID for the Front-Right module*/
    public static final int FR_DRIVE_ID = 3;
    /**Drive motor ID for the Back-Right module*/
    public static final int BR_DRIVE_ID = 7; 

    /*turning motors ids */
    /**Turning motor ID for the Front-Left module*/
    public static final int FL_TURN_ID = 2;
    /**Turning motor ID for the Back-Left module*/ 
    public static final int BL_TURN_ID = 6; 
    /**Turning motor ID for the Front-Right module*/
    public static final int FR_TURN_ID = 4; 
    /**Turning motor ID for the Back-Right module*/
    public static final int BR_TURN_ID = 8;

    public static final boolean GYRO_REVERSED = false;
  }

  /**
   * Constants for a NEO motor. Contains process variables
   */
  public static final class NeoMotorConstants {
      public static final double NEO_FREE_SPEED = 5676;
  }

  /**
   * Constants for all swerve modules. Contains process variables
   */
  public static final class ModuleConstants{
    /*pinion gear teeth */    
    public static final int DRIVE_MOTOR_TEETH = 14;  
    public static final boolean TURN_ENCODER_INVERTED = true;

    //Calculations for drive motor conversion factors and feed forwards
    public static final double DRIVE_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.NEO_FREE_SPEED / 60;
    public static final double WHEEL_DIAMETER_METERS = 0.0762/*Units.inchesToMeters(3.0)*/;
    public static final double WHEEL_CIRCUMFRENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (DRIVE_MOTOR_TEETH * 15);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFRENCE_METERS) / DRIVE_MOTOR_REDUCTION;
    
    public static final double DRIVE_ENCODER_POS_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
    / DRIVE_MOTOR_REDUCTION; // meters

    public static final double DRIVE_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
    / DRIVE_MOTOR_REDUCTION) / 60.0; // meters per second

    public static final double TURN_ENCODER_POS_FACTOR = (2 * Math.PI); // radians
    public static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second
    public static final double TURN_ENCODER_POS_MIN_INPUT = 0; // radians
    public static final double TURN_ENCODER_POS_MAX_INPUT = TURN_ENCODER_POS_FACTOR; // radians

    //drive motor PID
    public static final double DRIVE_P = 0.06;
    public static final double DRIVE_I = 0.0001;
    public static final double DRIVE_D = 0.0;
    public static final double DRIVE_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
    public static final double DRIVE_MIN_OUTPUT = -1;
    public static final double DRIVE_MAX_OUTPUT = 1;

    //turn motor PID
    public static final double TURN_P = 1;
    public static final double TURN_I = 0;
    public static final double TURN_D = 0;
    public static final double TURN_FF = 0;
    public static final double TURN_MIN_OUTPUT = -1;
    public static final double TURN_MAX_OUTPUT = 1;
    public static final IdleMode DRIVE_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode TURN_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 50; // amps
    public static final int TURN_MOTOR_CURRENT_LIMIT = 20; // amps
  }

  /**
   * Constants for use by the robot autonomously. Contains speeds and other process variables
   */
  public static class AutoContsants{
    public static final double AUTO_MAX_SPEED_MPS = 2;
    public static final double AUTO_MAX_ACCELERATION_MPS_SQUARED = 2;
    public static final double MAX_ANGULAR_SPEED_RPS = Math.PI;
    public static final double MAX_ANGULAR_SPEED_RPS_SQUARED = Math.PI;

    public static final double PX_CONTROLLER = 1;
    public static final double P_THETA_CONTROLLER = 1; 
    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ANGULAR_SPEED_RPS, MAX_ANGULAR_SPEED_RPS_SQUARED);        
  }

  /**
   * Constants for the arm subsystem. Contains IDs and other process variables
   */
  public static class ArmConstants{
    public static final boolean USE_LEFT_ENCODER = true;

    public static final double ANGLE_OFFSET_FROM_ZERO = 9.5;
    public static final double ANGLE_OFFSET_FROM_VERTICAL_DEGREES = 57.442;
    public static final double HEIGHT_OFFSET_FROM_GROUND_INCHES = 35.219;
    public static final double ARM_LENGTH_INCHES = 30.254;

    public static final int LEFT_MOTOR_ID = 31;

    public static final int RIGHT_MOTOR_ID = 32;

    public static final double ANGLE_CHECKTOLERANCE_DEGREES = 6.0;

    public static final double PID_kP = 0.001 * 1.4;
    public static final double PID_kI = 0.0;
    public static final double PID_kD = 0.0;
    public static final double PID_kFF = 0.0;
    public static final double ANGLE_CONE_INSURANCE = 20.0;
    public static final double ANGLE_CUBE_INSURANCE = 7.0;
    public static final double ANGLE_MID_OFFSET = 15.0;

    public static final double MANUAL_SPEED_MAX_DEGREESPERSECOND = 50.0;
    public static final boolean MANUAL_INVERT = false;

    public static final double ANGLE_CONE_HIGH = 201.182 - ANGLE_CONE_INSURANCE;
    public static final double ANGLE_CONE_MID = 224.367 - ANGLE_CONE_INSURANCE - ANGLE_MID_OFFSET;

    public static final double ANGLE_CUBE_HIGH = 201.182 - ANGLE_CUBE_INSURANCE;
    public static final double ANGLE_CUBE_MID = 224.367 - ANGLE_CUBE_INSURANCE - ANGLE_MID_OFFSET;


    /**
     * Calculates the angle required for the arm to rotate to in order to reach the desired height
     * @param heightInches Height above ground for the arm to end up
     * @return The angle for the arm to rotate to in order reach the desired height above the ground
     */
    public static double ANGLE_FROM_HEIGHT(double heightInches){
    double verticalDiff = heightInches - HEIGHT_OFFSET_FROM_GROUND_INCHES;
    double sideRatios = Math.abs(verticalDiff) / ARM_LENGTH_INCHES;
    double angleABS = (270.0 - (verticalDiff > 0.0 ? Math.asin(sideRatios) : -Math.asin(sideRatios)));
    double angle = (angleABS - ANGLE_OFFSET_FROM_VERTICAL_DEGREES) + ANGLE_OFFSET_FROM_ZERO;

    return angle;
  }

    /**
     * Calculates the angular velocity for the arm to rotate at in order to reach the desired linear velociy
     * @param metersPerSecond Linear velocity to convert to angular velocity
     * @return The angular velocity which corresponds to the linear velocity provided in degrees per second
     */
    public static double ANGULARVELOCITY_FROM_LINEAR(double metersPerSecond){
    double armCircumfrence = 2.0 * Units.inchesToMeters(ARM_LENGTH_INCHES) * Math.PI;
    double angularVelocity = (metersPerSecond / armCircumfrence) * 360.0;
    return angularVelocity;
  }

  }

  /**
   * Constants for the claw subsystem. Contains IDs and other process variables
   */
  public static class ClawConstants{
    public static final int WRIST_ID = GlobalConstants.PNEUMATICS_ID;
    public static final int WRIST_CHANNEL_FORWARD = 0;
    public static final int WRIST_CHANNEL_BACKWARD = 3;

    public static final int CLAW_ID = GlobalConstants.PNEUMATICS_ID;
    public static final int CLAW_CHANNEL_FORWARD = 4;
    public static final int CLAW_CHANNEL_BACKWARD = 1;

    public static final double DISTANCE_LIMIT_UPPER_MILLIMETERS = 60.0;
    public static final double DISTANCE_LIMIT_LOWER_MILLIMETERS = 0.0;

    public static final double ANGLE_WRIST_EXCLUSIONZONE_MIN = 206.595;
    public static final double ANGLE_WRIST_EXCLUSIONZONE_MAX = 229.523 + ArmConstants.ANGLE_OFFSET_FROM_ZERO;
  }

  /**
   * Constants for the spindexer subsystem. Contains IDs and other process variables
   */
  public static class SpindexerConstants{
    public static final int SPINDEXER_MOTOR_ID = 11;
    public static final double SPINDEXER_MOTOR_MAXOUTPUT = 0.25;

    public static final double SPINDEXER_GEARING_MOTORTOTABLE = 22.0 / 1.0;

    public static final double SPINDEXER_P = 0.08;
    public static final double SPINDEXER_I = 0.0;
    public static final double SPINDEXER_D = 0.0;
    public static final double SPINDEXER_FF = 0.0;

   }

  public static class CollectorConstants{
    public static int FLIP_MOTOR_ID = 0;
    public static int ROLLER_LEFT_MOTOR_ID = 0;
    public static int ROLLER_RIGHT_MOTOR_ID = 0;

    public static int GRIP_PISTON_ID = GlobalConstants.PNEUMATICS_ID;
    public static int PISTON_CHANNEL_FORWARD_ID = 0;
    public static int PISTON_CHANNEL_REVERSE_ID = 0;
  }

  /**
   * Constants for the lights subsystem. Contains colors and animations
   */
  public static class LightsConstants{
    public static final int[] COLOR_CONE = new int[] {255, 80, 0};
    public static final int[] COLOR_CUBE = new int[] {190, 20, 220};

    public static final int[] COLOR_ALLIANCE_RED = new int[] {128, 0, 0};
    public static final int[] COLOR_ALLIANCE_BLUE = new int[] {0, 0, 128};

    public static final LarsonAnimation ANIM_TRANSPORT_CONE = new LarsonAnimation(COLOR_CONE[0], COLOR_CONE[1], COLOR_CONE[2], 
      128, 0.1, 38, BounceMode.Front, 15, 0);
    public static final LarsonAnimation ANIM_TRANSPORT_CUBE = new LarsonAnimation(COLOR_CUBE[0], COLOR_CUBE[1], COLOR_CUBE[2], 
      128, 0.1, 38, BounceMode.Front, 15, 0);

    public static final ColorFlowAnimation ANIM_SCORE_CONE = new ColorFlowAnimation(COLOR_CONE[0], COLOR_CONE[1], COLOR_CONE[2],
      128, 0.6, 38, Direction.Forward, 0);
    public static final ColorFlowAnimation ANIM_SCORE_CUBE = new ColorFlowAnimation(COLOR_CUBE[0], COLOR_CUBE[1], COLOR_CUBE[2],
      128, 0.6, 38, Direction.Forward, 0);
  }
}
