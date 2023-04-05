package frc.robot.Utils;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;

/**
 * Wrapper class for {@link PIDController} to simplify usage.
 * 
 * <p>Allows automatic feedback acceptance as well as calculation consumption.
 * Simplifies all usage down to a single call of {@code AioPIDController.run()}
 */
public class AioPIDController extends PIDController{
    private DoubleSupplier feeder;
    private DoubleConsumer consumer;

    private double calculation = 0.0;

    private boolean scaleOutput = false;
    private double outputScalar = 1.0;

    /**
     * Constructs an AioPIDController that automatically performs calculations and sends the output to the appropriate
     * consumer
     * @param pidConstants Constants to construct the PID controller with
     * @param feeder The feedback device of the PID
     * @param consumer The consumer device of the PID
     */
    public AioPIDController(PIDConstants pidConstants, DoubleSupplier feeder, DoubleConsumer consumer){
        super(pidConstants.kP, pidConstants.kI, pidConstants.kD);
        this.feeder = feeder;
        this.consumer = consumer;
    }

    public void enableContinuousInput(double min, double max){
        super.enableContinuousInput(min, max);
    }

    public void disableContinuousInput(){
        super.disableContinuousInput();
    }

    public void disableOutputScaling(){
        scaleOutput = false;
        outputScalar = 1.0;
    }

    /**
     * Enables scaling of output
     */
    public void enableOutputScaling(double scalar){
        scaleOutput = true;
        outputScalar = scalar;
    }

    /**
     * Sets the value to multiply the calculated value
     * @param scalar The output the scalar
     */
    public void setOutputScalar(double scalar){
        outputScalar = scalar;
    }

    /**
     * Sets the goal of the PID
     * @param setpoint Goal for the PID
     */
    public void setSetpoint(double setpoint){
        super.setSetpoint(setpoint);
    }

    /**
     * Gets the last value outputted by the PID. Useful for debugging
     * 
     */
    public double getLastConsumedValue(){
        return calculation;
    }

    /**
     * Sets the feeder the AioPID to perform calculations on
     * @param feeder The new feeder
     */
    public void setFeeder(DoubleSupplier feeder){
        this.feeder = feeder;
    }

    /**
     * Sets the consumer of the AioPID calculations
     * @param consumer The new consumer
     */
    public void setConsumer(DoubleConsumer consumer){
        this.consumer = consumer;
    }

    /**
     * Performs a calculation with the PID controller on the fed value and sends output to the consumer
     */
    public void run(){
        calculation = super.calculate(feeder.getAsDouble()) * outputScalar;
        calculation *= scaleOutput ? outputScalar : 1.0;
        consumer.accept(calculation);
    }
}
