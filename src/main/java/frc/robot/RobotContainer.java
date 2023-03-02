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
import frc.robot.Utils.Constants;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Constants.ControllerConstants;
import frc.robot.Utils.Constants.GlobalConstants;
import frc.robot.Utils.Constants.SpindexerConstants;
import frc.robot.commands.CompressCMD;
import frc.robot.commands.SpindexerCMD;
import frc.robot.commands.ArmCMDS.ArmSubStationInTake;
import frc.robot.commands.ArmCMDS.Place;
import frc.robot.commands.ArmCMDS.ResetArm;
import frc.robot.commands.ArmCMDS.LowLevelCMDS.ManualSetPointControl;
import frc.robot.commands.ArmCMDS.LowLevelCMDS.SetArmAngle;
import frc.robot.commands.ArmCMDS.LowLevelCMDS.SetCubeOrCone;
import frc.robot.commands.ArmCMDS.NodeCMDS.HighNode;
import frc.robot.commands.ArmCMDS.NodeCMDS.MidNode;
import frc.robot.commands.ClawCMDS.LowLevelCMDS.ToggleClaw;
import frc.robot.commands.ClawCMDS.LowLevelCMDS.ToggleWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Compress;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Spindexer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //subsystems 
  public static DriveTrain drive = new DriveTrain();
  public static Claw claw = new Claw();
  public static Arm arm = new Arm(claw);
  public static Compress comp = new Compress();
  public static Spindexer spindexer = new Spindexer();
  public static Limelight limelight = new Limelight();
  public static LED led = new LED();
  

  //Contollers 
  CommandXboxController driveController = new CommandXboxController(ControllerConstants.DRIVE_CONTROLLER);
  CommandXboxController opController = new CommandXboxController(ControllerConstants.OP_CONTROLLER);

  SlewRateLimiter strafe = new SlewRateLimiter(5);
  SlewRateLimiter translate = new SlewRateLimiter(5);
  boolean fieldOriented = false;
  Trigger wristFlipTrigger = new Trigger(arm::isWristAllowedOut);

  //auto chooser
  private SendableChooser<Command> chooser;
  
  public RobotContainer() {
    
    
    
    
    // Configure the trigger bindings
    configureBindings();
   
   SetCubeOrCone commonCommand = new SetCubeOrCone(arm, opController);

    //drive buttons
  
       driveController.rightBumper().whileTrue(new RunCommand(() -> drive.setX(), drive));
    driveController.leftBumper().onTrue(new InstantCommand(() -> fieldOriented = !fieldOriented));
    driveController.start().onTrue(new InstantCommand(() -> drive.zeroHeading(), drive));


    // HIGH PLACEMENT
    opController.povUp().onTrue(new HighNode(arm, claw, opController));
    
    // MID PLACEMENT
    opController.povLeft().onTrue(new MidNode(arm, claw, opController));
    
    // ARM RESET
    opController.povDown().onTrue(new ResetArm(this));
    
    // MANUAL CONTROL
    armManualControl.whileTrue(new ManualSetPointControl(arm, opController));
    
    // INTAKE POSITION
    opController.rightBumper().onTrue(new ArmSubStationInTake(this)).onFalse(new ResetArm(this));

    // AUTO WRIST
    wristFlipTrigger.onTrue(Commands.runOnce(() -> claw.setManualWristControlAllowed(true))).whileFalse(
                                            Commands.runOnce(() -> claw.setManualWristControlAllowed(false)));

    // CLAW TOGGLE
    opController.x().onTrue(new ToggleClaw(claw));

    opController.b().onTrue(new ToggleWrist(claw));

    // SPINDEXER FORWARD
    opController.rightTrigger(ControllerConstants.OP_CONTROLLER_THRESHOLD_SPINDEXER).whileTrue(
                                                                    new RunAtSpeed(spindexer, 1.0, opController));
    // SPINDEXER REVERSE
    opController.leftTrigger(ControllerConstants.OP_CONTROLLER_THRESHOLD_SPINDEXER).whileTrue(
                       new RunAtSpeed(spindexer, -1.0, opController));
  }

   
   
   
   


    //default commands 
  

  

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
