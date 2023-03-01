// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Constants.ControllerConstants;
import frc.robot.Utils.Constants.GlobalConstants;
import frc.robot.Utils.Constants.SpindexerConstants;
import frc.robot.commands.CompressCMD;
import frc.robot.commands.SpindexerCMD;
import frc.robot.commands.ArmCMDS.Place;
import frc.robot.commands.ArmCMDS.ResetArm;
import frc.robot.commands.ArmCMDS.LowLevelCMDS.SetArmAngle;
import frc.robot.commands.ArmCMDS.LowLevelCMDS.SetCubeOrCone;
import frc.robot.commands.ClawCMDS.LowLevelCMDS.ToggleClaw;
import frc.robot.commands.ClawCMDS.LowLevelCMDS.ToggleWrist;
import frc.robot.commands.DriveCMDS.TranslateToTag;
import frc.robot.commands.DriveCMDS.TurnToTag;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Compress;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //subsystems 
  public static DriveTrain drive = new DriveTrain();
  public static Arm arm = new Arm();
  public static Claw claw = new Claw();
  public static Compress comp = new Compress();
  public static Spindexer spindexer = new Spindexer();
  public static Vision vision = new Vision();
 

  //Contollers 
  CommandXboxController driveController = new CommandXboxController(ControllerConstants.DRIVE_CONTROLLER);
  CommandXboxController opController = new CommandXboxController(ControllerConstants.OP_CONTROLLER);

  SlewRateLimiter strafe = new SlewRateLimiter(5);
  SlewRateLimiter translate = new SlewRateLimiter(5);
  boolean fieldOriented = false;
  
  //auto chooser
  private SendableChooser<Command> chooser;
  
  public RobotContainer() {
    
    
    
    
    // Configure the trigger bindings
    configureBindings();
   
   SetCubeOrCone commonCommand = new SetCubeOrCone(arm, opController);

    //drive buttons
    new JoystickButton(driveController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> drive.setX(),
            drive));
    new JoystickButton(driveController, XboxController.Button.kLeftBumper.value)
        .onTrue(new InstantCommand(
            () -> fieldOriented = !fieldOriented));
    new JoystickButton(driveController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> drive.zeroHeading(),
            drive));
    new JoystickButton(driveController, XboxController.Button.kA.value).whileTrue(new TurnToTag(vision, drive));
    new JoystickButton(driveController, XboxController.Button.kB.value).whileTrue(new TranslateToTag(vision, drive));

    //spindexer buttons
    new JoystickButton(opController, XboxController.Button.kLeftBumper.value)
    .whileTrue(new SpindexerCMD(SpindexerConstants.SPINDEXER_MOTOR_MAXOUTPUT));
    new JoystickButton(opController, XboxController.Button.kRightBumper.value)
    .whileTrue(new SpindexerCMD(-SpindexerConstants.SPINDEXER_MOTOR_MAXOUTPUT));

    //claw buttons
    /*new JoystickButton(m_operatorController, XboxController.Button.kA.value)
    .onTrue(new ClawCmd());
    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
    .onTrue(new WristCmd());*/

   
   
   
   // HIGH PLACEMENT
    opController.povUp().onTrue(new Place(Commands.waitUntil(arm::getNOTHolding), 
                                                new SetArmAngle(arm, ArmConstants.ANGLE_CONE_HIGH, ArmConstants.ANGLE_CUBE_HIGH), 
                                                commonCommand));
  
    // MID PLACEMENT
    opController.povLeft().onTrue(new Place(Commands.waitUntil(arm::getNOTHolding), 
                                                new SetArmAngle(arm, ArmConstants.ANGLE_CONE_MID, ArmConstants.ANGLE_CUBE_MID),
                                                commonCommand));

    // ARM RESET
    opController.povDown().onTrue(new ResetArm(this));

    opController.povRight().onTrue(new SetArmAngle(arm, 275.0));



    // CLAW TOGGLE
    opController.x().onTrue(new ToggleClaw(claw));

    opController.b().onTrue(new ToggleWrist(claw));


    //default commands 
  

  }

  /**
   * Use this method ito define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
