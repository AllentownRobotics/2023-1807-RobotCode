package frc.robot.commands.DriveCMDS;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TurnTarget extends CommandBase {

    private PIDController kturningPID = new PIDController(0.015, 0, 0.0001);
    private PIDController kStrafingPID = new PIDController(0.0175, 0, 0.000);
    private PIDController kDrivingPID = new PIDController(0.0175, 0, 0);

    
    
    private DriveTrain s_Swerve;


    public TurnTarget(DriveTrain s_Swerve) {

        kturningPID.enableContinuousInput(-180, 180);
        kturningPID.setTolerance(1);
        kStrafingPID.setTolerance(0.5);
        kDrivingPID.setTolerance(0.5);



        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

    }

    @Override
    public void execute() {


        s_Swerve.drive(0, 0, kturningPID.calculate(s_Swerve.getHeadingDegrees(), 0), true, false);
    }


    @Override
    public boolean isFinished() {
        if (Math.abs(s_Swerve.getHeadingDegrees()) % 360 >= 358 || Math.abs(s_Swerve.getHeadingDegrees()) % 360 <= 2) {
            return true;
        } else {return false;}

    }
}
