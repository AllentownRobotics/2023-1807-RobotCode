// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static class GlobalConstants{
    public static final int PNEUMATICS_ID = 14;
  }

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 1;
    public static final int OPERATOR_CONTROLLER_PORT = 0;

    public static final double OPERATOR_CONTROLLER_THRESHOLD_SPINDEXER = 0.08;
  }

  public static class ArmConstants{
    public static final boolean USE_LEFT_ENCODER = true;

    public static final double ANGLE_OFFSET_FROM_ZERO = 9.5;
    public static final double ANGLE_OFFSET_FROM_VERTICAL_DEGREES = 57.442 + ANGLE_OFFSET_FROM_ZERO;
    public static final double HEIGHT_OFFSET_FROM_GROUND_INCHES = 35.219;
    public static final double ARM_LENGTH_INCHES = 30.254;

    public static final int LEFT_MOTOR_ID = 31;

    public static final int RIGHT_MOTOR_ID = 32;

    public static final double PID_kP = 0.15;
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
    public static final int CLAW_CHANNEL_FORWARD = 1;
    public static final int CLAW_CHANNEL_BACKWARD = 4;

    public static final double ANGLE_WRIST_FLIPPOINT = 190.0 + ArmConstants.ANGLE_OFFSET_FROM_ZERO;
  }

  public static class SpindexerConstants{
    public static final int SPINDEXER_MOTOR_ID = 0;

    public static final double SPINDEXER_MOTOR_MAXOUTPUT = 0.5;

    public static final double COLORSENSOR_THRESHOLD_CUBE = 0.04;
    public static final double COLORSENSOR_THRESHOLD_CONE = 0.04;

    public static final Color COLORSENSOR_COLOR_CUBE = new Color(190, 20, 220);
    public static final Color COLORSENSOR_COLOR_CONE = new Color(230, 220, 0);
  }

  public static class CompressorConstants{
    public static int COMPRESSOR_ID = GlobalConstants.PNEUMATICS_ID;

    public static double COMPRESSOR_RANGE_MIN = 60.0;
    public static double COMPRESSOR_RANGE_MAX = 120.0;
  }

  public static class CollectorConstants{
    public static final int TOP_MOTOR_ID = 0;
    public static final int BOTTOM_MOTOR_ID = 0;

    public static final int COLLECTOR_PISTON_ID = GlobalConstants.PNEUMATICS_ID;
    public static final int COLLECTOR_PISTON_CHANNEL_FORWARD = 0;
    public static final int COLLECTOR_PISTON_CHANNEL_BACKWARD = 0;

    // SUBJECT TO CHANGE
    public static final int BELT_MOTOR_ID = 0;

    public static final double COLLECTOR_SPEED_MAX = 0.5;
  }
} 