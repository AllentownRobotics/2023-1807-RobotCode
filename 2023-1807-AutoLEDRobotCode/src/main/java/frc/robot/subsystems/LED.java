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
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  CANdle candle;
  CANdleConfiguration config;
  StrobeAnimation coneStrobeAnim;
  ColorFlowAnimation coneFlowAnim;
  StrobeAnimation cubeStrobeAnim;
  ColorFlowAnimation cubeFlowAnim;
  RainbowAnimation rainbowAnim;
  SingleFadeAnimation idleFadeAnim;
  LarsonAnimation coneLarsonAnim;
  LarsonAnimation cubeLarsonAnim;
  Direction direction;
  BounceMode bounce;
  WPI_Pigeon2 pigeon;

  public static int TEAM_R;
  public static int TEAM_G;
  public static int TEAM_B;

  public static Timer timer;

  double endgameBright;
  double tilt;

  public static int animNumber;
  
  public LED() {
    candle = new CANdle(10);
  
    config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    candle.configAllSettings(config);

    direction = Direction.Forward;
    bounce = BounceMode.Front;

    candle = new CANdle(10);
    config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    candle.configAllSettings(config);

    pigeon = new WPI_Pigeon2(9);

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

  public void setAnimNumber(int animNumber) {
    LED.animNumber = animNumber;
  }
  
  public void IdleAnim() {
    candle.animate(idleFadeAnim, 0);
  }
  
  public void ConeReqAnim() {
    candle.setLEDs(ColorConstants.CONE_R, ColorConstants.CONE_G, ColorConstants.CONE_B);
  }
  
  public void CubeReqAnim() {
    candle.setLEDs(ColorConstants.CUBE_R, ColorConstants.CUBE_G, ColorConstants.CUBE_B);
  }
  
  public void ConeTransportAnim() {
    candle.animate(coneLarsonAnim, 1);
  }
  
  public void CubeTransportAnim() {
    candle.animate(cubeLarsonAnim, 2);
  }

  public void ConeScoreAnim() {
    
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
    IdleAnim();
  
        timer.stop();
        timer.reset();
  }

  public void CubeScoreAnim() {
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
IdleAnim();

timer.stop();
    timer.reset();
}
  // always change brightness to the correct value
public void EndGameAnim() {
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

public double SetEndgameBright() {

  // use the pythagorean theorem to account for pitch and roll in one variable
  tilt = (Math.sqrt((Math.pow(pigeon.getRoll(), 2)) + Math.pow(pigeon.getPitch(), 2)));
  
  while (true) {
    // function to set the brightness (logistic function I picked, you can pick your own)
    endgameBright = (-1 / (1 + (50 * Math.pow(Math.E, -.7 * tilt)))) + 1 ;
  return endgameBright;
  }
}

  public void NoAnim() {

    // clears animation slots and sets LEDs to 0
    candle.setLEDs(0, 0, 0);

    candle.clearAnimation(0);
    candle.clearAnimation(1);
    candle.clearAnimation(2);
    candle.clearAnimation(3);
    candle.clearAnimation(4);
  }

  public static void TranslateReqAndTransport() {
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
    switch (animNumber) {
      case AnimNumberConstants.IDLE_ANIM_NUMBER: IdleAnim();
      break;
      case AnimNumberConstants.CONE_REQ_ANIM_NUMBER: ConeReqAnim();
      break;
      case AnimNumberConstants.CUBE_REQ_ANIM_NUMBER: CubeReqAnim();
      break;
      case AnimNumberConstants.CONE_TRANSPORT_ANIM_NUMBER: ConeTransportAnim();
      break;
      case AnimNumberConstants.CUBE_TRANSPORT_ANIM_NUMBER: CubeTransportAnim();
      break;
      case AnimNumberConstants.CONE_SCORE_ANIM_NUMBER: ConeScoreAnim(); 
      break;
      case AnimNumberConstants.CUBE_SCORE_ANIM_NUMBER: CubeScoreAnim();
      break;
      case AnimNumberConstants.ENDGAME_ANIM_NUMBER: EndGameAnim();
      break;
      case AnimNumberConstants.RESET_ANIM_NUMBER: NoAnim();
      break;
  }
}
}
