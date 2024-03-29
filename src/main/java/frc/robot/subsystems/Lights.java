// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Enums.CycleState;
import frc.robot.Utils.LightAnimation;

public class Lights extends SubsystemBase {
  CANdle candle = new CANdle(10);

  public static int[] allianceColor = new int[] {255, 255, 255};

  DoubleSupplier tiltSupplier;

  public LightAnimation currentAnimation = LightAnimation.nullAnim;

  static Lights instance = null;

  /**
   * Creates a new Lights. 
   * NOTE: This method should not be manually called. Instead,
   * use the singleton instance by calling the static method {@link Lights#getInstance()} 
   */
  public Lights() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    candle.configAllSettings(config);

    tiltSupplier = DriveTrain.getInstance()::getTilt;
    setAnimation(LightAnimation.nullAnim);
  }

  /**
   * Gets the singleton instance of the lights. If no instance exists one is automatically created.
   * @return The singleton instance of the lights
   */
  public static Lights getInstance(){
    if (instance == null){
      instance = new Lights();
    }
    return instance;
  }

  /**
   * Sets the animation on the lights using the provided animation
   * @param animation New animation
   */
  public Lights setAnimation(LightAnimation animation){
    currentAnimation = animation;
    
    animation.run(candle);
    
    if (animation == LightAnimation.endgame){
      candle.setLEDs(allianceColor[0], allianceColor[1], allianceColor[2]);
    }

    if (animation == LightAnimation.nullAnim){
      candle.clearAnimation(0);
      candle.clearAnimation(1);
      candle.clearAnimation(2);
      candle.clearAnimation(3);
      candle.clearAnimation(4);
    }

    return this;
  }

  public void setLEDs(int r, int g, int b){
    candle.setLEDs(r, g, b);
  }

  /**
   * Calculates the brightness for the lights to scale to during the leveling in endgame
   * using a modified sigmoid function
   * @return The brightness scalar
   */
  public double calculateBrightness(){
    double tilt = tiltSupplier.getAsDouble();

    return Math.min(1.0, -2.0 * Math.abs((1.6 / (1 + Math.exp(-tilt / 3.17))) - 0.8) + 1.6);
  }

  /**
   * Handles the transition to a new cycle state
   * @param newState New cycle state to transition to
   */
  public void transitionToNewCycleState(CycleState newState){
    setAnimation(LightAnimation.nullAnim);

    if (newState.equals(CycleState.Transporting)){
      if (currentAnimation == LightAnimation.coneRequest){ 
        setAnimation(LightAnimation.coneTransport);
        return;
      }
      if (currentAnimation == LightAnimation.cubeRequest){
        setAnimation(LightAnimation.cubeTransport);
        return;
      }
    }
    if (newState.equals(CycleState.Scoring)){
      if (currentAnimation == LightAnimation.coneTransport){
        setAnimation(LightAnimation.coneScore);
        return;
      }
      if (currentAnimation == LightAnimation.cubeTransport){
        setAnimation(LightAnimation.cubeScore);
        return;
      }
    }
  }

  @Override
  public void periodic() {}
}
