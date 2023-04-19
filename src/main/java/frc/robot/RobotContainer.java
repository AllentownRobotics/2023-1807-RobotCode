// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Utils.Constants.ArmConstants;
import frc.robot.Utils.Constants.AutoContsants;
import frc.robot.Utils.Constants.ControllerConstants;
import frc.robot.Utils.Constants.DriveConstants;
import frc.robot.Utils.Constants.LightsConstants;
import frc.robot.Utils.Enums.CycleState;
import frc.robot.Utils.Enums.GamePiece;
import frc.robot.commands.CompressCMD;
import frc.robot.commands.ArmCMDS.ArmSubStationInTake;
import frc.robot.commands.ArmCMDS.AutoPlace;
import frc.robot.commands.ArmCMDS.GroundPickup;
import frc.robot.commands.ArmCMDS.ResetArm;
import frc.robot.commands.ArmCMDS.NodeCMDS.HighNode;
import frc.robot.commands.ArmCMDS.NodeCMDS.MidNode;
import frc.robot.commands.AutoCMDS.AutoLevel;
import frc.robot.commands.ClawCMDS.LowLevelCMDS.ToggleClaw;
import frc.robot.commands.ClawCMDS.LowLevelCMDS.ToggleWrist;
import frc.robot.commands.CollectorCMDS.CollectorGrab;
import frc.robot.commands.CollectorCMDS.CollectorIn;
import frc.robot.commands.CollectorCMDS.CollectorOut;
import frc.robot.commands.CollectorCMDS.CollectorSpit;
import frc.robot.commands.DriveCMDS.ChaseCube;
import frc.robot.commands.DriveCMDS.DriveCMD;
import frc.robot.commands.DriveCMDS.PseudoNodeTargeting;
import frc.robot.commands.LightCMDS.BlinkColor;
import frc.robot.commands.LightCMDS.FlashColor;
import frc.robot.commands.SpindexerCMDS.RunAtSpeed;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Compress;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Spindexer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static GamePiece desiredGamePiece = GamePiece.Cone;

  //Subsystems
  public DriveTrain drive = DriveTrain.getInstance();
  public Claw claw = Claw.getInstance();
  public Arm arm = Arm.getInstance();
  public Compress comp = Compress.getInstance();
  public Spindexer spindexer = Spindexer.getInstance();
  public Collector collector = Collector.getInstance();
  public Lights light = Lights.getInstance();
  public Limelight limelight = Limelight.getInstance();
  
  //Contollers 
  CommandXboxController driveController = new CommandXboxController(ControllerConstants.DRIVE_CONTROLLER);
  CommandXboxController opController = new CommandXboxController(ControllerConstants.OP_CONTROLLER);

  //Triggers
  Trigger wristFlipTrigger = new Trigger(arm::isWristAllowedOut);
  Trigger autoGrabTrigger = new Trigger(() -> collector.isAutoGrabEnabled() && collector.pieceInRange());
  Trigger armManualControl = new Trigger(() -> Math.abs(opController.getLeftY()) >= 0.1);
  Trigger collectionRollersControl = new Trigger(() -> Math.abs(opController.getRightY()) >= 0.1);
  Trigger visionPoseTrigger = new Trigger(Limelight::goodForEstimator);
  Trigger driverStationTrigger = new Trigger(DriverStation::isDSAttached);

  //Commands HashMap
  HashMap<String, Command> commandsMap = new HashMap<>();

  boolean fieldOriented = true;
  
  SwerveAutoBuilder autoBuilder = generateAutoBuilder();

  //auto chooser
  private SendableChooser<Command> chooser = new SendableChooser<Command>();
  
  public RobotContainer() {
    populateCommandMap();

    comp.setDefaultCommand(new CompressCMD(comp));
    drive.setDefaultCommand(new DriveCMD(driveController, () -> fieldOriented));
    
    drive.resetEncoders();
    
    chooser.setDefaultOption("Mid Basic", autoBuilder.fullAuto(PathPlanner.loadPathGroup("MidHighConeEngage", 2.0, 2.0)));
    chooser.addOption("Wall Basic", autoBuilder.fullAuto(PathPlanner.loadPathGroup("WallHighConeEngage", 3.0, 3.0)));
    chooser.addOption("Loadzone Basic", autoBuilder.fullAuto(PathPlanner.loadPathGroup("LoadzoneHighConeEngage", 3.0, 3.0)));
    chooser.addOption("Loadzone 2 Piece Low", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Loadzone2PieceLow", 2.5, 2.5)));
    chooser.addOption("Loadzone 2 Piece High Low", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Loadzone2PieceHighLow", 2.5, 2.5)));
    chooser.addOption("Mid 1.5 Piece", autoBuilder.fullAuto(PathPlanner.loadPathGroup("MidHighConeGrabEngage", 2.0, 2.0)));
    chooser.addOption("Loadzone 2.5 Piece (Unreliable)", autoBuilder.fullAuto(PathPlanner.loadPathGroup("2.5 Piece Test", 2.5, 2.5)));
    chooser.addOption("Wall 2.5 Piece", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Wall2Piece", 2.5, 2.5)));

    SmartDashboard.putData("Auto Chooser", chooser);

    // Configure the trigger bindings
    configureBindings();

    // Non Controller based trigger bindings
    wristFlipTrigger.onTrue(Commands.runOnce(() -> claw.setManualWristControlAllowed(true))).onFalse(
      Commands.runOnce(() -> claw.setManualWristControlAllowed(false)));

    autoGrabTrigger.onTrue(new CollectorGrab());

    visionPoseTrigger.whileTrue(Commands.repeatingSequence(
      Commands.runOnce(() -> drive.updateEstimatorWithVision(limelight.botPoseFieldSpace())),
      Commands.waitSeconds(5.0)));

    driverStationTrigger.whileFalse(new BlinkColor(new int[]{186, 52, 0}, 1.5, 1.5).ignoringDisable(true)).onTrue(
    new BlinkColor(new int[]{0, 186, 0}, 0.1, 0.1).withTimeout(1.5).andThen(Commands.runOnce(() -> light.setLEDs(Lights.allianceColor[0], Lights.allianceColor[1], Lights.allianceColor[2]))).ignoringDisable(true));
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
    driveController.x().whileTrue(new AutoLevel());

    driveController.rightBumper() 
        .whileTrue(new RunCommand(
            () -> drive.setX(),
            drive));
    driveController.start()
        .onTrue(new InstantCommand(
            () -> drive.zeroHeading(),
            drive));

    driveController.leftStick().onTrue(Commands.runOnce(() -> fieldOriented = !fieldOriented));

    driveController.leftBumper().whileTrue(new ChaseCube(driveController, opController));

    // TARGETING
    driveController.leftTrigger().whileTrue(new PseudoNodeTargeting(drive, driveController, opController)).onFalse(
      Commands.runOnce(() -> opController.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

    // HIGH PLACEMENT
    opController.povUp().onTrue(Commands.runOnce(() -> light.transitionToNewCycleState(CycleState.Scoring)).andThen(
      new HighNode(arm, claw)));
    
    // MID PLACEMENT
    opController.povLeft().onTrue(Commands.runOnce(() -> light.transitionToNewCycleState(CycleState.Scoring)).andThen(
      new MidNode(arm, claw)));
    
    // ARM RESET
    opController.povDown().onTrue(new ResetArm());
    
    // MANUAL CONTROL
    armManualControl.onTrue(Commands.runOnce(() -> arm.setAutomaticMode(false))).whileTrue(
      Commands.run(() -> arm.setManualSpeed(ArmConstants.MANUAL_SPEED_MAX_DEGREESPERSECOND * MathUtil.applyDeadband(opController.getLeftY(), 0.1)))).onFalse(
      Commands.runOnce(() -> arm.setAutomaticMode(true)).andThen(Commands.runOnce(() -> arm.setDesiredAngle(arm.getArmAngle()))));
    
    // INTAKE POSITION
    opController.rightBumper().onTrue(new ArmSubStationInTake(this)).onFalse(new ResetArm().andThen(
      Commands.runOnce(() -> {int[] color = desiredGamePiece.equals(GamePiece.Cone) ? LightsConstants.COLOR_CONE : LightsConstants.COLOR_CUBE; light.setLEDs(color[0], color[1], color[2]);})));

    // CLAW TOGGLE
    opController.x().onTrue(new ToggleClaw(claw));

    // WRIST TOGGLE
    opController.b().onTrue(new ToggleWrist(claw));

    // SPINDEXER FORWARD
    opController.rightTrigger(ControllerConstants.OP_CONTROLLER_THRESHOLD_SPINDEXER).whileTrue(
                                   new RunAtSpeed(spindexer, opController::getRightTriggerAxis));
    // SPINDEXER REVERSE
    opController.leftTrigger(ControllerConstants.OP_CONTROLLER_THRESHOLD_SPINDEXER).whileTrue(
                       new RunAtSpeed(spindexer, (() -> -1.0 * opController.getLeftTriggerAxis())));

    // CONE REQUEST
    opController.start().onTrue(Commands.runOnce(() -> desiredGamePiece = GamePiece.Cone).andThen(
      new BlinkColor(LightsConstants.COLOR_CONE, 0.5, 0.2).alongWith(
      limelight.LightOn().andThen(
      limelight.setLimePipe()))));

    // CUBE REQUEST
    opController.back().onTrue(Commands.runOnce(() -> desiredGamePiece = GamePiece.Cube).andThen(
      new BlinkColor(LightsConstants.COLOR_CUBE, 0.5, 0.2).alongWith(
      limelight.LightOff().andThen(
      limelight.setApril2DPipe()))));

    opController.leftBumper().onTrue(new CollectorOut()).onFalse(new CollectorIn());
    
    opController.y().onTrue(Commands.runOnce(() -> collector.toggleGripState()));

    collectionRollersControl.whileTrue(new CollectorSpit(opController::getRightY));

    opController.a().onTrue(Commands.runOnce(() -> collector.setAutoGrabUsage(true))).onFalse(
      Commands.runOnce(() -> collector.setAutoGrabUsage(false)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return chooser.getSelected();
  }

  /**
   * Populates the command hashmap for use with the autoBuilder
   */
  private void populateCommandMap(){
    commandsMap.put("resetArm", new ResetArm());
    commandsMap.put("autoScoreHighCone", new AutoPlace(arm, claw, ArmConstants.ANGLE_CONE_HIGH));
    commandsMap.put("autoScoreHighCube", new AutoPlace(arm, claw, ArmConstants.ANGLE_CUBE_HIGH));
    commandsMap.put("pickUpFromGround", new GroundPickup(this));
    commandsMap.put("autoLevel", new AutoLevel());
    commandsMap.put("collectOut", new CollectorOut());
    commandsMap.put("collectIn", new CollectorIn());
    commandsMap.put("collectGrab", Commands.runOnce(() -> collector.setAutoGrabUsage(true)).andThen(Commands.waitUntil(() -> autoGrabTrigger.getAsBoolean()).andThen(new CollectorGrab())));
    commandsMap.put("spit", new CollectorSpit(() -> 2.0 / 3.0).withTimeout(1.0));
    commandsMap.put("flipGyro", Commands.runOnce(() -> drive.flipGyro()));
  }

  /**
   * Generates an auto builder
   */
  private SwerveAutoBuilder generateAutoBuilder(){
    return new SwerveAutoBuilder(
      drive::getEstimatedPose, 
      drive::resetOdometry, 
      DriveConstants.DRIVE_KINEMATICS,
      new PIDConstants(AutoContsants.PX_CONTROLLER, 0, 0),
      new PIDConstants(AutoContsants.P_THETA_CONTROLLER, 0, 0),
      drive::setModuleStatesNoOptimize,
      commandsMap, 
      true,
      drive);
  }

  public void putSmartDashboardData(){
    SmartDashboard.putBoolean("Field Oriented", fieldOriented);
  }
}