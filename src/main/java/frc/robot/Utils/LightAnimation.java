package frc.robot.Utils;

import com.ctre.phoenix.led.CANdle;

import frc.robot.subsystems.Lights;

import static frc.robot.Utils.Constants.LightsConstants.*;

/**
 * Interface which serves as a lambda wrapper for all light animations.
 * <p>To create custom animations create an arrow function which takes in a CANdle object. 
 * To use, simply call {@link LightAnimation#run(CANdle)}
 */
public interface LightAnimation {
    /**
     * Runs the animation
     * @param candle the CANdle object for the lights to run off of
     */
    void run(CANdle candle);

    public static LightAnimation coneRequest = (candle) -> candle.setLEDs(COLOR_CONE[0], COLOR_CONE[1], COLOR_CONE[2]);
    public static LightAnimation coneTransport = (candle) -> candle.animate(ANIM_TRANSPORT_CONE, 1);
    public static LightAnimation coneScore = (candle) -> candle.animate(ANIM_SCORE_CONE, 3);
    public static LightAnimation cubeRequest = (candle) -> candle.setLEDs(COLOR_CUBE[0], COLOR_CUBE[1], COLOR_CUBE[2]);
    public static LightAnimation cubeTransport = (candle) -> candle.animate(ANIM_TRANSPORT_CUBE, 2);
    public static LightAnimation cubeScore = (candle) -> candle.animate(ANIM_SCORE_CUBE, 3);
    public static LightAnimation endgame = (candle) -> candle.configBrightnessScalar(Lights.getInstance().calculateBrightness());
    public static LightAnimation nullAnim = (candle) -> candle.setLEDs(0, 0, 0);
}