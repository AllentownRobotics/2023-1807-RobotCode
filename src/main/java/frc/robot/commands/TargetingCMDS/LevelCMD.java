package frc.robot.commands.TargetingCMDS;


import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class LevelCMD extends CommandBase {
        
    private DriveTrain s_Swerve;

    //private RollingAverage pitchAverage = new RollingAverage(10);
    PIDController kLevelingPID = new PIDController(0.035, 0, 0);

    
    public LevelCMD(DriveTrain s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        kLevelingPID.setIntegratorRange(-3, 3);
        //kLevelingPID.setTolerance(0.5);

    }

    @Override
    public void execute() {

        s_Swerve.levelSet(-kLevelingPID.calculate(s_Swerve.getRoll(), 0));
    }

    @Override
    public boolean isFinished() {
        
        return false;
        
    }

}
