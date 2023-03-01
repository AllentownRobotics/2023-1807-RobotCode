// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

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
public static final double MAX_SPEED_MPS = 2;
public static final double MAX_ANGLE_SPEED = 2 * Math.PI; //Radians per sec

//chassis config
public static final double TRACK_WIDTH = Units.inchesToMeters(26); 
public static final double WHEEL_BASE = Units.inchesToMeters(26);

public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
    new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
    new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
    new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
    new Translation2d(-WHEEL_BASE /2, -TRACK_WIDTH /2));

//angular offsets of modules from chassis
public static final double FL_CHASSIS_OFFSET = -Math.PI / 2;
public static final double FR_CHASSIS_OFFSET = 0.0; 
public static final double BL_CHASSIS_OFFSET = Math.PI; 
public static final double BR_CHASSIS_OFFSET = Math.PI / 2;

//SPARK CAN IDS
/*driving motor ids */
public static final int FL_DRIVE_ID = 1;
public static final int BL_DRIVE_ID = 5;
public static final int FR_DRIVE_ID = 3;
public static final int BR_DRIVE_ID = 7; 

/*turning motors ids */
public static final int FL_TURN_ID = 2; 
public static final int BL_TURN_ID = 6; 
public static final int FR_TURN_ID = 4; 
public static final int BR_TURN_ID = 8;

public static final boolean GYRO_REVERSED = false;


}

public static final class NeoMotorConstants {
    public static final double NEO_FREE_SPEED = 5676;
  }

  public static final class ModuleConstants{
    /*pinion gear teeth */    
    public static final int DRIVE_MOTOR_TEETH = 13;  

    public static final boolean TURN_ENCODER_INVERTED = true;

//Calculations for drive motor conversion factors and feed forwards
    public static final double DRIVE_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.NEO_FREE_SPEED / 60;
    public static final double WHEEL_DIAMETER_METERS = 0.0762;
    public static final double WHEEL_CIRCUMFRENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (DRIVE_MOTOR_TEETH * 15);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVE_MOTOR_FREE_SPEED_RPS* WHEEL_CIRCUMFRENCE_METERS) / DRIVE_MOTOR_REDUCTION;

    public static final double DRIVE_ENCODER_POS_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
    / DRIVE_MOTOR_REDUCTION; // meters
    public static final double DRIVE_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
    / DRIVE_MOTOR_REDUCTION) / 60.0; // meters per second

    public static final double TURN_ENCODER_POS_FACTOR = (2 * Math.PI); // radians
    public static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

    public static final double TURN_ENCODER_POS_MIN_INPUT = 0; // radians
    public static final double TURN_ENCODER_POS_MAX_INPUT = TURN_ENCODER_POS_FACTOR; // radians

    //drive motor PID
    public static final double DRIVE_P = 0.04;
    public static final double DRIVE_I = 0;
    public static final double DRIVE_D = 0;
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

//Auto Constnts 
public static class AutoContsants{
    
   public static final double AUTO_MAX_SPEED_MPS = 2;
    public static final double AUTO_MAX_ACCELERATION_MPS_SQUARED = 2;
    public static final double MAX_ANGULAR_SPEED_RPS = Math.PI;
    public static final double MAX_ANGULAR_SPEED_RPS_SQUARED = Math.PI;
 
    public static final double PX_CONTROLLER = 1;
    public static final double PY_CONTROLLER = 1;
    public static final double P_THETA_CONTROLLER = 1; 
    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ANGULAR_SPEED_RPS, MAX_ANGULAR_SPEED_RPS_SQUARED);        
}

public static class ArmConstants{
  
  public static final boolean USE_LEFT_ENCODER = true;

  public static final double ANGLE_OFFSET_FROM_ZERO = 9.5;
  public static final double ANGLE_OFFSET_FROM_VERTICAL_DEGREES = 57.442 + ANGLE_OFFSET_FROM_ZERO;
  public static final double HEIGHT_OFFSET_FROM_GROUND_INCHES = 35.219;
  public static final double ARM_LENGTH_INCHES = 30.254;

  public static final int LEFT_MOTOR_ID = 31;

  public static final int RIGHT_MOTOR_ID = 32;

  public static final double PID_kP = 0.12;
  public static final double PID_kI = 0.0;
  public static final double PID_kD = 0.0;
  public static final double PID_kFF = 0.0;
  
  public static final double ANGLE_CONE_INSURANCE = 5.0;
  public static final double ANGLE_MANUAL_SPEED_MAX_DEGREESPERSECOND = 120.0;
  public static final double ANGLE_MANUAL_INPUT_MODIFIER = ANGLE_MANUAL_SPEED_MAX_DEGREESPERSECOND * 0.02;

  public static final double ANGLE_CONE_HIGH = 195.37664 - ANGLE_CONE_INSURANCE - ANGLE_OFFSET_FROM_ZERO;
  public static final double ANGLE_CONE_MID = 210.2488 - ANGLE_CONE_INSURANCE - ANGLE_OFFSET_FROM_ZERO;

  public static final double ANGLE_CUBE_HIGH = 195.37664 - ANGLE_OFFSET_FROM_ZERO;
  public static final double ANGLE_CUBE_MID = 210.2488 - ANGLE_OFFSET_FROM_ZERO;

  public static final double ANGLE_FROM_HEIGHT(double heightInches, boolean insure){
    double verticalDiff = heightInches - HEIGHT_OFFSET_FROM_GROUND_INCHES;
    double sideRatios = Math.abs(verticalDiff) / ARM_LENGTH_INCHES;
    double angleABS = (270.0 - verticalDiff > 0.0 ? Math.asin(sideRatios) : -Math.asin(sideRatios));
    double angle = angleABS - ANGLE_OFFSET_FROM_VERTICAL_DEGREES;

    angle -= insure ? ANGLE_CONE_INSURANCE : 0.0;

    return angle;
  }
}
public static class ClawConstants{
  public static final int WRIST_ID = GlobalConstants.PNEUMATICS_ID;
  public static final int WRIST_CHANNEL_FORWARD = 0;
  public static final int WRIST_CHANNEL_BACKWARD = 3;

  public static final int CLAW_ID = GlobalConstants.PNEUMATICS_ID;
  public static final int CLAW_CHANNEL_FORWARD = 4;
  public static final int CLAW_CHANNEL_BACKWARD = 1;

  public static final double ANGLE_WRIST_FLIPPOINT = 190.0 + ArmConstants.ANGLE_OFFSET_FROM_ZERO;
}

public static class SpindexerConstants{
  public static final int SPINDEXER_MOTOR_ID = 0;
  public static final double SPINDEXER_MOTOR_MAXOUTPUT = 0.5;

 }
 public static class ColorConstants {

  public static final int CONE_R = 255;
  public static final int CONE_G = 80;
  public static final int CONE_B = 0;
  
  public static final int CUBE_R = 190;
  public static final int CUBE_G = 20;
  public static final int CUBE_B = 220;

  public static final int RED_TEAM_R = 128;
  public static final int RED_TEAM_G = 0;
  public static final int RED_TEAM_B = 0;

  public static final int BLUE_TEAM_R = 0;
  public static final int BLUE_TEAM_G = 0;
  public static final int BLUE_TEAM_B = 128;

}
public static class AnimNumberConstants {

  public static final int IDLE_ANIM_NUMBER = 0;

  public static final int CONE_REQ_ANIM_NUMBER = 1;
  public static final int CUBE_REQ_ANIM_NUMBER = 2;

  public static final int CONE_TRANSPORT_ANIM_NUMBER = 3;
  public static final int CUBE_TRANSPORT_ANIM_NUMBER = 4;

  public static final int CONE_SCORE_ANIM_NUMBER = 5;
  public static final int CUBE_SCORE_ANIM_NUMBER = 6;

  public static final int ENDGAME_ANIM_NUMBER = 7;
  public static final int RESET_ANIM_NUMBER = 8;
}

}
