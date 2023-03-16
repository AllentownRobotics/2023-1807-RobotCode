// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCMDS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class AutoAlignNodes extends CommandBase {
  ProfiledPIDController rotationController;
  ProfiledPIDController translationController;
  ProfiledPIDController strafeController;

  DriveTrain drive;

  double horizontalPosition;
  double backPosition;

  public AutoAlignNodes(DriveTrain drive, double horizontalPosition, double backPosition) {
    addRequirements(drive);
    this.drive = drive;
    this.horizontalPosition = horizontalPosition;
    this.backPosition = backPosition;

    initPIDs();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] coords = Limelight.targetPoseRobotSpace();

    double rotationSpeed = rotationController.calculate(drive.getHeadingDegrees());
    double translateSpeed = translationController.calculate(coords[2]);
    double strafeSpeed = strafeController.calculate(coords[0]);

    drive.driveFromComponentSpeeds(translateSpeed, strafeSpeed, rotationSpeed);
  }


  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotationController.atSetpoint() && translationController.atSetpoint() && strafeController.atSetpoint();
  }

  private void initPIDs(){
    rotationController = new ProfiledPIDController(0.015, 0, 0, new TrapezoidProfile.Constraints(180.0, 120.0));
    rotationController.enableContinuousInput(-180.0, 180.0);
    rotationController.setGoal(new TrapezoidProfile.State(0.0, 0.0));
    rotationController.setTolerance(2.0);

    translationController = new ProfiledPIDController(1.0, 0, 0, new TrapezoidProfile.Constraints(2.0, 1.5));
    translationController.setGoal(new TrapezoidProfile.State(backPosition, 0.0));
    translationController.setTolerance(0.05);

    strafeController = new ProfiledPIDController(1.0, 0, 0, new TrapezoidProfile.Constraints(2.0, 1.5));
    strafeController.setGoal(new TrapezoidProfile.State(horizontalPosition, 0.0));
    strafeController.setTolerance(0.05);
  }
}
