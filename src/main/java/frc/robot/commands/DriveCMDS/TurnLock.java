package frc.robot.commands.DriveCMDS;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class TurnLock extends CommandBase {

    private PIDController kturningPID = new PIDController(0.015, 0, 0.0001);

    
    private DriveTrain s_Swerve;
    private CommandXboxController drivecontroller;
    private double rotation;

    public TurnLock(DriveTrain s_Swerve, CommandXboxController drivecontroller, double rotation) {

        kturningPID.enableContinuousInput(-180, 180);
        kturningPID.setTolerance(1);

        this.rotation = rotation;
        this.s_Swerve = s_Swerve;
        this.drivecontroller = drivecontroller;
        addRequirements(s_Swerve);

    }

    @Override
    public void execute() {
        s_Swerve.drive(
            MathUtil.applyDeadband(drivecontroller.getLeftY(), 0.3),
            MathUtil.applyDeadband(drivecontroller.getLeftX(), 0.3),
            kturningPID.calculate(s_Swerve.getHeadingDegrees(), rotation),  
            true);
    }

}
