// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Utils.Constants.*;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  static CANdle candle;
  CANdleConfiguration config;
  static ColorFlowAnimation flowAnim;
  static SingleFadeAnimation idleFadeAnim;
  static LarsonAnimation coneLarsonAnim;
  static LarsonAnimation cubeLarsonAnim;
  Direction direction;
  BounceMode bounce;
  static WPI_Pigeon2 pigeon;

  public static int TEAM_R;
  public static int TEAM_G;
  public static int TEAM_B;

  public int GAMEPIECE_R;
  public int GAMEPIECE_G;
  public int GAMEPIECE_B;

  public static Timer timer;

  static double endgameBright;
  static double tilt;

  public static int animNumber;

  private DriveTrain driveTrain;
  
  public LED(DriveTrain driveTrain) {
    candle = new CANdle(10);
  
    config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    candle.configAllSettings(config);

    direction = Direction.Forward;
    bounce = BounceMode.Front;

    pigeon = driveTrain.getGyro();

    // using alliance color to set colors
    DriverStation.Alliance alliance = DriverStation.getAlliance();
  
    if (alliance == DriverStation.Alliance.Blue) {
      TEAM_R = ColorConstants.BLUE_TEAM_R;
      TEAM_G = ColorConstants.BLUE_TEAM_G;
      TEAM_B = ColorConstants.BLUE_TEAM_B;
    }
    else if (alliance == DriverStation.Alliance.Red) {
      TEAM_R = ColorConstants.RED_TEAM_R;
      TEAM_G = ColorConstants.RED_TEAM_G;
      TEAM_B = ColorConstants.RED_TEAM_B;
    }
    else { 
      TEAM_R = 255;
      TEAM_G = 255;
      TEAM_B = 255;
    }

    // establishing animations
    idleFadeAnim = new SingleFadeAnimation
    (TEAM_R, TEAM_G, TEAM_B, 
    0, .25, 38, 0);

    flowAnim = new ColorFlowAnimation
    (GAMEPIECE_R, GAMEPIECE_G, GAMEPIECE_B,
    128, .6, 38, direction, 0);

    coneLarsonAnim = new LarsonAnimation
    (ColorConstants.CONE_R, ColorConstants.CONE_G, ColorConstants.CONE_B, 
    128, .1, 38, bounce, 15, 0);

    cubeLarsonAnim = new LarsonAnimation
    (ColorConstants.CUBE_R, ColorConstants.CUBE_G, ColorConstants.CUBE_B, 
    128, .1, 38, bounce, 15, 0);


    timer = new Timer();

    endgameBright = 0;
    
    candle.setLEDs(0, 0, 0);
  }

  public void setAllianceColor() {
    candle.setLEDs(TEAM_R, TEAM_G, TEAM_B);
    }

  public static void setAnimNumber(int animNumber) {
    LED.animNumber = animNumber;
  }
  
  public static void IdleAnim() {
    candle.animate(idleFadeAnim, 0);
  }
  
  public static void ConeReqAnim() {
    candle.setLEDs(ColorConstants.CONE_R, ColorConstants.CONE_G, ColorConstants.CONE_B);
  }
  
  public static void CubeReqAnim() {
    candle.setLEDs(ColorConstants.CUBE_R, ColorConstants.CUBE_G, ColorConstants.CUBE_B);
  }
  
  public static void ConeTransportAnim() {
    candle.animate(coneLarsonAnim, 1);
  }
  
  public static void CubeTransportAnim() {
    candle.animate(cubeLarsonAnim, 2);
  }

  public void ScoringAnimInit() {
    candle.animate(flowAnim, 3);
  }


  // always changes brightness to the correct value
public void NotBalancedAnim() {
    candle.configBrightnessScalar(CalculateEndgameBrightness());
}

public boolean isBalanced(){
  return CalculateTilt() <= 2.5;
}

public void setLEDs(int r, int g, int b) {
  candle.setLEDs(r, g, b);
}
public double CalculateTilt() {
  return (Math.sqrt((Math.pow(pigeon.getRoll(), 2)) + Math.pow(pigeon.getPitch(), 2)));
}
public static double CalculateEndgameBrightness() {

  // use the pythagorean theorem to account for pitch and roll in one variable
  tilt = (Math.sqrt((Math.pow(pigeon.getRoll(), 2)) + Math.pow(pigeon.getPitch(), 2)));
  
  while (true) {
    // function to set the brightness (logistic function I picked, you can pick your own)
  return (-1 / (1 + (50 * Math.pow(Math.E, -.7 * tilt)))) + 1 ;
  }
}

  public static void NoAnim() {

    // clears animation slots and sets LEDs to 0
    candle.setLEDs(0, 0, 0);

    candle.clearAnimation(0);
    candle.clearAnimation(1);
    candle.clearAnimation(2);
    candle.clearAnimation(3);
    candle.clearAnimation(4);
  }

  public void TranslateReqAndTransport() {
    if (animNumber == AnimNumberConstants.CONE_REQ_ANIM_NUMBER) {
      animNumber = AnimNumberConstants.CONE_TRANSPORT_ANIM_NUMBER;
      GAMEPIECE_R = ColorConstants.CONE_R;
      GAMEPIECE_G = ColorConstants.CONE_G;
      GAMEPIECE_B = ColorConstants.CONE_B;

    } else if (animNumber == AnimNumberConstants.CUBE_REQ_ANIM_NUMBER) {
      animNumber = AnimNumberConstants.CUBE_TRANSPORT_ANIM_NUMBER;
      GAMEPIECE_R = ColorConstants.CUBE_R;
      GAMEPIECE_G = ColorConstants.CUBE_G;
      GAMEPIECE_B = ColorConstants.CUBE_B;

    } else if (animNumber == AnimNumberConstants.CONE_TRANSPORT_ANIM_NUMBER) {
      animNumber = AnimNumberConstants.CONE_SCORE_ANIM_NUMBER;
      GAMEPIECE_R = ColorConstants.CONE_R;
      GAMEPIECE_G = ColorConstants.CONE_G;
      GAMEPIECE_B = ColorConstants.CONE_B;

    } else if (animNumber == AnimNumberConstants.CUBE_TRANSPORT_ANIM_NUMBER) {
      animNumber = AnimNumberConstants.CUBE_SCORE_ANIM_NUMBER;
      GAMEPIECE_R = ColorConstants.CUBE_R;
      GAMEPIECE_G = ColorConstants.CUBE_G;
      GAMEPIECE_B = ColorConstants.CUBE_B;
    }
  }

  @Override
  public void periodic() {
  }
}
