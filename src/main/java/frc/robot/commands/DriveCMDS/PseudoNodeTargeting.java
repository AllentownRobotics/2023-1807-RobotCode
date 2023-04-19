package frc.robot.commands.DriveCMDS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;

public class PseudoNodeTargeting extends SequentialCommandGroup {
    
    private PIDController kturningPID = new PIDController(0.015, 0.0, 0.0001);
    private PIDController kStrafingPID = new PIDController(0.018, 0.0, 0.0);
    
    private PIDController kDrivingPID = new PIDController(0.018, 0, 0);

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
    }

}