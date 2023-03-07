package frc.robot.commands.DriveCMDS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class PseudoNodeTargeting extends SequentialCommandGroup {
    
    private PIDController kturningPID = new PIDController(0.015, 0, 0.0001);
    private PIDController kStrafingPID = new PIDController(0.02, 0, 0.000);
    private PIDController kDrivingPID = new PIDController(0.02, 0, 0);


    private SlewRateLimiter translate = new SlewRateLimiter(5);

    public PseudoNodeTargeting(DriveTrain m_drive, CommandXboxController controller) {
        kturningPID.enableContinuousInput(-180, 180);
        kturningPID.setTolerance(1);
        kStrafingPID.setTolerance(0.5);
        kDrivingPID.setTolerance(0.5);
    
        addRequirements(m_drive);

        addCommands(
            //alt 2
            //m_drive.gyroSanityCheck(),
            //new RunCommand(() -> m_drive.drive(0, 0, kturningPID.calculate(m_drive.getHeading(), 180), true)).until(() -> kturningPID.atSetpoint()),
            // alterenatively 
            //new RunCommand(() -> m_drive.drive(0, 0, kturningPID.calculate(Limelight.targetRelPos[5], 0), false)).until(() -> kStrafingPID.atSetpoint())
            //new RunCommand(() -> m_drive.drive(0, kStrafingPID.calculate(Limelight.x, 0), kturningPID.calculate(m_drive.getHeading(), 180), false))
            //.until(() -> kStrafingPID.atSetpoint())//,
            //Forward Motion Sequence
            //new RunCommand(() -> m_drive.drive(kDrivingPID.calculate(Limelight.y, yTarget), 0, 0, false)).until(() -> kDrivingPID.atSetpoint())
            new TurnTarget(m_drive),
            new RunCommand(() -> m_drive.drive(translate.calculate(MathUtil.applyDeadband(controller.getLeftY(), 0.3)), kStrafingPID.calculate(Limelight.x, 0), kturningPID.calculate(m_drive.getHeadingDegrees(), 0), false)
            , m_drive)
            //new ForwardCubeTarget(m_drive)
            
        );

    }

}