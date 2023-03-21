package frc.robot.Utils;

import java.util.function.DoubleSupplier;

/**
 * Custom trapezoid profile with a ramp down on only one side
 */
public class FauxTrapezoidProfile {
    private double errorTolerance;
    private double maxVelocity;
    private double minVelocity;
    private double deccelRate;

    private double goal;

    /**
     * Creates a trapezoid profile with a ramp on only one side with the following constraints
     * @param maxVelocity The max allowed velocity
     * @param deccelRate Rate for the velocity to ramp down at
     * @param minVelocity The min allowed velocity
     * @param errorTolerance Tolerance in which position error will return 0 velocity
     */
    public FauxTrapezoidProfile(double maxVelocity, double deccelRate, double minVelocity, double errorTolerance){
        this.errorTolerance = errorTolerance;
        this.maxVelocity = maxVelocity;
        this.minVelocity = minVelocity;
        this.deccelRate = Math.abs(deccelRate);
    }

    /**
     * Calculates the appropriate feedforward goals for the mechanism to use
     * @param position Current position of mechanism
     * @return The calculated feedforward goals
     */
    public FeedForwardFeeder calculate(double position){
        double error = goal - position;
        DoubleSupplier calculator = error < 0.0 ? () -> calcNegError(error) : () -> calcPosError(error);

        double velocity = Math.abs(error) <= errorTolerance ? 0.0 : calculator.getAsDouble();

        double acceleration = velocity == maxVelocity || velocity == minVelocity ? 0.0 : -deccelRate * Math.signum(error);
        return new FeedForwardFeeder(goal, velocity, acceleration);
    }

    private double calcNegError(double error){
        return Math.min(Math.max(-maxVelocity, error * deccelRate), -minVelocity);
    }

    private double calcPosError(double error){
        return Math.max(Math.min(maxVelocity, error * deccelRate), minVelocity);
    }

    /**
     * Sets the desired end state of the profile
     * @param goal Desired end state
     */
    public void setGoal(double goal){
        this.goal = goal;
    }

    public double getGoal(){
        return goal;
    }

    public void setErrorTolerance(double errorTolerance){
        this.errorTolerance = errorTolerance;
    }

    public void setMaxVelocity(double maxVelocity){
        this.maxVelocity = maxVelocity;
    }

    public void setDeccelRate(double deccelRate){
        this.deccelRate = deccelRate;
    }

    public void setMinVelocity(double minVelocity){
        this.minVelocity = minVelocity;
    }

    /**
     * Container class for the values used by an {@code ArmFeedForward} when calculating
     */
    public class FeedForwardFeeder{
        public double position;
        public double velocity;
        public double acceleration;

        public FeedForwardFeeder(double position, double velocity, double acceleration){
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }

        /**
         * Applies a conversion to the contained values
         * @param conversionFunction the conversion to apply
         */
        public void convert(ConversionLambda conversionFunction){
           position = conversionFunction.convert(position);
           velocity = conversionFunction.convert(velocity);
           acceleration = conversionFunction.convert(acceleration);
        }
    }
}
