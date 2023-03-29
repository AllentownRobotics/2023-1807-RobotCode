package frc.robot.commands.DriveCMDS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class HighNodeTarget extends SequentialCommandGroup {
    
    private PIDController kturningPID = new PIDController(0.015, 0, 0.0001);
    private PIDController kStrafingPID = new PIDController(0.018, 0.000, 0.000);
    
    private PIDController kDrivingPID = new PIDController(0.011, 0, 0); //reduced for safety testing

    private SlewRateLimiter translate = new SlewRateLimiter(5);

    private LinearFilter filter = LinearFilter.movingAverage(5); 

    private double deadband = 0.01;

    /**
     * Sequential command group which rotates the robot to face the alliance wall, then strafe the robot
     * to align with the primary in-view limelight target. Runs until interrupted
     */
    public HighNodeTarget(DriveTrain m_drive, CommandXboxController driveController, CommandXboxController opController) {
        kturningPID.enableContinuousInput(-180, 180);
        kturningPID.setTolerance(1);
        kStrafingPID.setTolerance(0.5);
        kDrivingPID.setTolerance(0.5);
        
        addRequirements(m_drive);

        addCommands(
            new TurnTarget(m_drive),
            /*Strafe Only, jitter cut*/ //Commands.run(() -> m_drive.drive(translate.calculate(MathUtil.applyDeadband(driveController.getLeftY(), 0.3)), MathUtil.applyDeadband(kStrafingPID.calculate(Limelight.x, 0), deadband), kturningPID.calculate(m_drive.getHeadingDegrees(), 0), false)
           /*Og*/ //Commands.run(() -> m_drive.drive(translate.calculate(MathUtil.applyDeadband(driveController.getLeftY(), 0.3)), kStrafingPID.calculate(Limelight.x, 0), kturningPID.calculate(m_drive.getHeadingDegrees(), 0), false)
            /*Distance Applied (0 for high)*/ Commands.run(() -> m_drive.drive(kDrivingPID.calculate(Limelight.y, 0), kStrafingPID.calculate(Limelight.x, 0), kturningPID.calculate(m_drive.getHeadingDegrees(), 0), false)

            /*Distance Applied (0 for high), jitter cut*/ //Commands.run(() -> m_drive.drive(MathUtil.applyDeadband(kDrivingPID.calculate(Limelight.y, 0), deadband), MathUtil.applyDeadband(kStrafingPID.calculate(Limelight.x, 0), deadband), kturningPID.calculate(m_drive.getHeadingDegrees(), 0), false)

            , m_drive).alongWith(Commands.waitUntil(() -> (kStrafingPID.atSetpoint() && kDrivingPID.atSetpoint())).andThen(Commands.run(() -> opController.getHID().setRumble(RumbleType.kBothRumble, 0.5)))));
    }

}