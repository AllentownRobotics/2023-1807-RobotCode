package frc.robot.commands.TargetingCMDS;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class ClassicalTargeting extends CommandBase {


    
    private boolean fieldOriented = true;
    
    private DriveTrain s_Swerve;

    private SlewRateLimiter strafe = new SlewRateLimiter(5);
    private SlewRateLimiter translate = new SlewRateLimiter(5);
    private CommandXboxController controller;
    private PIDController kturningPID = new PIDController(0.02, 0, 0);

    public ClassicalTargeting(CommandXboxController controller, boolean fieldOriented, DriveTrain s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.fieldOriented = fieldOriented;
        this.controller = controller;
    }

    @Override
    public void execute() {
        s_Swerve.drive(
            translate.calculate(MathUtil.applyDeadband(-controller.getLeftY(), 0.3)),
            strafe.calculate(MathUtil.applyDeadband(-controller.getLeftX(), 0.3)),
            kturningPID.calculate(Limelight.x, 0),
            fieldOriented);
    }


    @Override
    public boolean isFinished() {
      return false;
    }
}
