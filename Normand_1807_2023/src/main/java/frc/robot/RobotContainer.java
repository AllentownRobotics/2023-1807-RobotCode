// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SpindexConstants;
import frc.robot.commands.ClawCmd;
import frc.robot.commands.CollectCmd;
import frc.robot.commands.CompressCmd;
import frc.robot.commands.SpindexerCmd;
import frc.robot.commands.WristCmd;
import frc.robot.commands.Arm.High;
import frc.robot.commands.Arm.Low;
import frc.robot.commands.Arm.Mid;
import frc.robot.commands.Arm.Spindex;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public static Spindexer m_Spindexer;
  public static Claw m_Claw;
  public static Compress m_Compressor;
  public static Collector m_Collector;
  public static Arm m_Arm;
  private boolean fieldOriented = false;
  private boolean speedUp = false;
  public static boolean cubeMode = false;
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  SlewRateLimiter strafe = new SlewRateLimiter(5);
  SlewRateLimiter translate = new SlewRateLimiter(5);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_Spindexer = new Spindexer();
    m_Claw = new Claw();
    m_Compressor = new Compress();
    m_Collector = new Collector();
    m_Arm = new Arm();

    m_Compressor.setDefaultCommand(new CompressCmd());
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                translate.calculate(MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.3)),
                strafe.calculate(MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.3)),
                MathUtil.applyDeadband(-m_driverController.getRightX(), 0.3),
                fieldOriented, speedUp),
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
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(
            () -> speedUp = !speedUp));
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    //spindexer buttons
    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
    .whileTrue(new SpindexerCmd(SpindexConstants.spindexSpeed));
    new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
    .whileTrue(new SpindexerCmd(-SpindexConstants.spindexSpeed));

    //claw buttons
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
    .onTrue(new ClawCmd());
    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
    .onTrue(new WristCmd());

    //collector buttons
    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
    .whileTrue(new CollectCmd(CollectorConstants.collectSpeed));
    new JoystickButton(m_operatorController, XboxController.Button.kX.value)
    .whileTrue(new CollectCmd(-CollectorConstants.collectSpeed));

    //arm buttons
    new POVButton(m_operatorController, 0).onTrue(new High(cubeMode));
    new POVButton(m_operatorController, 90).onTrue(new Mid(cubeMode));
    new POVButton(m_operatorController, 270).onTrue(new Low(cubeMode));
    new POVButton(m_operatorController, 180).onTrue(new Spindex(cubeMode));
    new JoystickButton(m_operatorController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> cubeMode = !cubeMode));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Trajectory autoTrajectory) {
    // Create config for trajectory
    /*TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);*/

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        autoTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(autoTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false,false));
  }
}
