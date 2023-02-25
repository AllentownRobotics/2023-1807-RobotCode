// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SpindexConstants;
import frc.robot.commands.CollectCmd;
import frc.robot.commands.CompressCmd;
import frc.robot.commands.SpindexerCmd;
import frc.robot.commands.Auto.*;
import frc.robot.commands.Drive.TranslateToTag;
import frc.robot.commands.Drive.TurnToTag;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final static DriveSubsystem m_robotDrive = new DriveSubsystem();
  public static Spindexer m_Spindexer;
  public static Claw m_Claw;
  public static Compress m_Compressor;
  public static Collector m_Collector;
  public static Arm m_Arm;
  public static Vision m_vision;
  public static boolean cubeMode = false;
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  SlewRateLimiter strafe = new SlewRateLimiter(5);
  SlewRateLimiter translate = new SlewRateLimiter(5);
  boolean fieldOriented = false;

  private SendableChooser<Command> chooser;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_Spindexer = new Spindexer();
    m_Claw = new Claw();
    m_Compressor = new Compress();
    m_Collector = new Collector();
    m_Arm = new Arm(m_Claw);
    m_vision = new Vision();

    m_Compressor.setDefaultCommand(new CompressCmd());
    // Configure the button bindings
    configureButtonBindings();

    //chooser
    chooser = new SendableChooser<Command>();
    chooser.setDefaultOption("Default Auto", new ExampleAuto(m_robotDrive, m_Arm));
    chooser.addOption("Test", new ExampleAuto2(m_robotDrive));
    chooser.addOption("tia", new TiaAuto(m_robotDrive, m_Arm));
    Shuffleboard.getTab("Autos").add(chooser);


    // Configure default commands
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
          () -> m_robotDrive.drive(
              translate.calculate(MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.3)),
              strafe.calculate(MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.3)),
              MathUtil.applyDeadband(-m_driverController.getRightX(), 0.3),
              fieldOriented),
          m_robotDrive));  
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //drive buttons
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .onTrue(new InstantCommand(
            () -> fieldOriented = !fieldOriented));
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));
    new JoystickButton(m_driverController, XboxController.Button.kA.value).whileTrue(new TurnToTag(m_vision, m_robotDrive));
    new JoystickButton(m_driverController, XboxController.Button.kB.value).whileTrue(new TranslateToTag(m_vision, m_robotDrive));

    //spindexer buttons
    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
    .whileTrue(new SpindexerCmd(SpindexConstants.spindexSpeed));
    new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
    .whileTrue(new SpindexerCmd(-SpindexConstants.spindexSpeed));

    //claw buttons
    /*new JoystickButton(m_operatorController, XboxController.Button.kA.value)
    .onTrue(new ClawCmd());
    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
    .onTrue(new WristCmd());*/

    //collector buttons
    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
    .whileTrue(new CollectCmd(CollectorConstants.collectSpeed));
    new JoystickButton(m_operatorController, XboxController.Button.kX.value)
    .whileTrue(new CollectCmd(-CollectorConstants.collectSpeed));

    //arm buttons
    /*new POVButton(m_operatorController, 0).onTrue(new High(cubeMode, m_Arm));
    new POVButton(m_operatorController, 90).onTrue(new Mid(cubeMode, m_Arm));
    new POVButton(m_operatorController, 270).onTrue(new Low(cubeMode, m_Arm));
    new POVButton(m_operatorController, 180).onTrue(new Spindex(cubeMode));
    new JoystickButton(m_operatorController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> cubeMode = !cubeMode));*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
