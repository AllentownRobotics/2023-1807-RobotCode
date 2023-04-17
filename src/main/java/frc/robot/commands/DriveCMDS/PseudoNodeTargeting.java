package frc.robot.commands.DriveCMDS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class PseudoNodeTargeting extends SequentialCommandGroup {
    
    private PIDController kturningPID = new PIDController(0.015, 0.0, 0.0001);
    private PIDController kStrafingPID = new PIDController(0.018, 0.0, 0.0);
    
    private PIDController kDrivingPID = new PIDController(0.018, 0, 0);

    private SlewRateLimiter translate = new SlewRateLimiter(5);

    /**
     * Sequential command group which rotates the robot to face the alliance wall, then strafe the robot
     * to align with the primary in-view limelight target. Runs until interrupted
     */
    public PseudoNodeTargeting(DriveTrain m_drive, CommandXboxController driveController, CommandXboxController opController) {
        kturningPID.enableContinuousInput(-180, 180);
        kturningPID.setTolerance(1);
        kStrafingPID.setTolerance(0.5);
        kDrivingPID.setTolerance(0.5);
        
        addRequirements(m_drive);

        addCommands(
            new TurnTarget(m_drive),
            new StrafeTarget(driveController));
            /*Commands.run(() -> m_drive.drive(translate.calculate(MathUtil.applyDeadband(driveController.getLeftY(), 0.3)), kStrafingPID.calculate(MathUtil.applyDeadband(Limelight.x, 0.0)), kturningPID.calculate(m_drive.getHeadingDegrees(), 0), false, false)
            , m_drive).alongWith(Commands.waitUntil(() -> kStrafingPID.atSetpoint()).andThen(Commands.run(() -> opController.getHID().setRumble(RumbleType.kBothRumble, 0.5)))));*/

            //Commands.run(() -> m_drive.drive(translate.calculate(MathUtil.applyDeadband(driveController.getLeftY(), 0.3)), kStrafingPID.calculate(MathUtil.applyDeadband((Limelight.x), .01), kturningPID.calculate(m_drive.getHeadingDegrees(), 0), false)
            //, m_drive).alongWith(Commands.waitUntil(() -> kStrafingPID.atSetpoint()).andThen(Commands.run(() -> opController.getHID().setRumble(RumbleType.kBothRumble, 0.5)))));
    
    }

}