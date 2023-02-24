// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Arm.ManualSetPointControl;
import frc.robot.commands.Arm.ResetArm;
import frc.robot.commands.Arm.RotateArmToSetPoint;
import frc.robot.commands.Claw.ToggleClaw;
import frc.robot.commands.Claw.ToggleWrist;
import frc.robot.commands.Collector.Collect;
import frc.robot.commands.Compressor.Compress;
import frc.robot.commands.Spindexer.Spindex;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Cmprsr;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Spindexer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  final Claw claw = new Claw();
  public final Arm arm = new Arm();
  final Cmprsr compressor = new Cmprsr();
  //final Spindexer spindexer = new Spindexer();
  //final Collector collector = new Collector();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    arm.setDefaultCommand(new ManualSetPointControl(arm, operatorController));

    compressor.setDefaultCommand(new Compress(compressor));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // HIGH PLACEMENT
    operatorController.povUp().onTrue(new RotateArmToSetPoint(arm, operatorController, 
                                          ArmConstants.ANGLE_CONE_HIGH, ArmConstants.ANGLE_CUBE_HIGH));
    
    // MID PLACEMENT
    operatorController.povLeft().onTrue(new RotateArmToSetPoint(arm, operatorController, 
                                          ArmConstants.ANGLE_CONE_MID, ArmConstants.ANGLE_CUBE_MID));

    // ARM RESET
    operatorController.povDown().onTrue(new ResetArm(this));

    operatorController.povRight().onTrue(new RotateArmToSetPoint(arm, 275.0));



    // CLAW TOGGLE
    operatorController.x().onTrue(new ToggleClaw(claw));

    operatorController.b().onTrue(new ToggleWrist(claw));

    // SPINDEXER FORWARD
    //operatorController.rightTrigger(OperatorConstants.OPERATOR_CONTROLLER_THRESHOLD_SPINDEXER).whileTrue(new Spindex(spindexer, 1.0));
    // SPINDEXER REVERSE
    //operatorController.leftTrigger(OperatorConstants.OPERATOR_CONTROLLER_THRESHOLD_SPINDEXER).whileTrue(new Spindex(spindexer, -1.0));

    // COLLECT
    //operatorController.a().whileTrue(new Collect(collector));
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