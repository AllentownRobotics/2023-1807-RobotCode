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
  StrobeAnimation coneStrobeAnim;
  static ColorFlowAnimation coneFlowAnim;
  StrobeAnimation cubeStrobeAnim;
  static ColorFlowAnimation cubeFlowAnim;
  static SingleFadeAnimation idleFadeAnim;
  static LarsonAnimation coneLarsonAnim;
  static LarsonAnimation cubeLarsonAnim;
  Direction direction;
  BounceMode bounce;
  static WPI_Pigeon2 pigeon;

  public static int TEAM_R;
  public static int TEAM_G;
  public static int TEAM_B;

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
    0, .15, 68, 0);

    coneFlowAnim = new ColorFlowAnimation
    (ColorConstants.CONE_R, ColorConstants.CONE_G, ColorConstants.CONE_B,
    128, .25, 68, direction, 0);

    coneLarsonAnim = new LarsonAnimation
    (ColorConstants.CONE_R, ColorConstants.CONE_G, ColorConstants.CONE_B, 
    128, .1, 68, bounce, 15, 0);

    cubeFlowAnim = new ColorFlowAnimation
    (ColorConstants.CUBE_R, ColorConstants.CUBE_G, ColorConstants.CUBE_B, 
    128, .25, 68, direction, 0);

    cubeLarsonAnim = new LarsonAnimation
    (ColorConstants.CUBE_R, ColorConstants.CUBE_G, ColorConstants.CUBE_B, 
    128, .1, 68, bounce, 15, 0);


    timer = new Timer();

    endgameBright = 0;
    
    candle.setLEDs(0, 0, 0);
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

  public static void ConeScoreAnim() {
    
    timer.start();
    candle.animate(coneFlowAnim, 3);
  
    Timer.delay(2);
    for (int i = 0; i < 3; i++) {

    candle.setLEDs(ColorConstants.CONE_R, ColorConstants.CONE_G, ColorConstants.CONE_B);

    Timer.delay(.1);
    candle.setLEDs(0, 0, 0);

    Timer.delay(.1);
    }

    candle.setLEDs(0, 0, 0);
    setAnimNumber(AnimNumberConstants.IDLE_ANIM_NUMBER);
  
        timer.stop();
        timer.reset();
  }

  public static void CubeScoreAnim() {
    timer.start();
candle.animate(cubeFlowAnim, 4);

Timer.delay(2);

for (int i = 0; i < 3; i++) {

  candle.setLEDs(ColorConstants.CUBE_R, ColorConstants.CUBE_G, ColorConstants.CUBE_B);

  Timer.delay(.1);
  candle.setLEDs(0, 0, 0);

  Timer.delay(.1);
  }

Timer.delay(.8);
setAnimNumber(AnimNumberConstants.IDLE_ANIM_NUMBER);

timer.stop();
    timer.reset();
}
  // always changes brightness to the correct value
public static void EndGameAnim() {
  while (true) {
  SetEndgameBright();
  candle.configBrightnessScalar(endgameBright);
  candle.setLEDs(TEAM_R, TEAM_G, TEAM_B);

  while (tilt < 2.5) {
    timer.start();

    candle.setLEDs(TEAM_R, TEAM_G, TEAM_B);

    Timer.delay(.5);

    candle.setLEDs(0, 0, 0);

    Timer.delay(.5);
  }
  }
}

public static double SetEndgameBright() {

  // use the pythagorean theorem to account for pitch and roll in one variable
  tilt = (Math.sqrt((Math.pow(pigeon.getRoll(), 2)) + Math.pow(pigeon.getPitch(), 2)));
  
  while (true) {
    // function to set the brightness (logistic function I picked, you can pick your own)
    endgameBright = (-1 / (1 + (50 * Math.pow(Math.E, -.7 * tilt)))) + 1 ;
  return endgameBright;
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

    } else if (animNumber == AnimNumberConstants.CUBE_REQ_ANIM_NUMBER) {
      animNumber = AnimNumberConstants.CUBE_TRANSPORT_ANIM_NUMBER;

    } else if (animNumber == AnimNumberConstants.CONE_TRANSPORT_ANIM_NUMBER) {
      animNumber = AnimNumberConstants.CONE_SCORE_ANIM_NUMBER;

    } else if (animNumber == AnimNumberConstants.CUBE_TRANSPORT_ANIM_NUMBER) {
      animNumber = AnimNumberConstants.CUBE_SCORE_ANIM_NUMBER;
    }
  }

  @Override
  public void periodic() {
  }
}
