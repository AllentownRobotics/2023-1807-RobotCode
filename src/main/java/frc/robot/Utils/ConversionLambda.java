package frc.robot.Utils;

import edu.wpi.first.math.util.Units;

/**
 * Wrapper class which stores arrow functions of the {@code Units} class' conversion functions
 */
public interface ConversionLambda {
    double convert(double value);

    public static ConversionLambda degreesToRadians = (v) -> Units.degreesToRadians(v);
    public static ConversionLambda radiansToDegrees = (v) -> Units.radiansToDegrees(v);
}
