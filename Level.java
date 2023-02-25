package frc.robot.Commands;

import frc.robot.RollingAverage;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class Level extends CommandBase {
        
    private DriveSubsystem s_Swerve;

    private RollingAverage pitchAverage = new RollingAverage(10);
    PIDController kLevelingPID = new PIDController(0.035, 0, 0);

    
    public Level(DriveSubsystem s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        kLevelingPID.setIntegratorRange(-3, 3);
        //kLevelingPID.setTolerance(0.5);

    }

    @Override
    public void execute() {

        /*pitchAverage.add(s_Swerve.getRoll()); //ADD GET ROLL SHIT TO DRIVESUBSYSTEM... and the get everythings
        //s_Swerve.levelSet(0.5);
        if (pitchAverage.getAverage() > -1 && pitchAverage.getAverage() < 1){
            s_Swerve.setX();
        } else {s_Swerve.levelSet(-kLevelingPID.calculate(s_Swerve.getRoll(), 0)); //ALSO ADD THIS
        }*/
        s_Swerve.levelSet(-kLevelingPID.calculate(s_Swerve.getRoll(), 0));
    }

    @Override
    public boolean isFinished() {
        
        return false;
        
    }

}
