package frc.robot.commands.DrivingCMDS;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class DriveCMD extends CommandBase {

    private double rot;
    private double xSpeed;
    private double ySpeed;

    
    private boolean fieldOriented;
    
    private DriveTrain s_Swerve;

    private SlewRateLimiter strafe = new SlewRateLimiter(5);
    private SlewRateLimiter translate = new SlewRateLimiter(5);
    private CommandXboxController controller;

    public DriveCMD(CommandXboxController controller, boolean fieldOriented, DriveTrain s_Swerve) {
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
            MathUtil.applyDeadband(-controller.getRightX(), 0.3),
            fieldOriented);
    }
}
