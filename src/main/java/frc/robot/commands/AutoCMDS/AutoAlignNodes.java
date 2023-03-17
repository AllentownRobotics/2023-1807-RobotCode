// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCMDS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class AutoAlignNodes extends CommandBase {
  ProfiledPIDController rotationController;
  ProfiledPIDController translationController;
  ProfiledPIDController strafeController;

  SwerveDriveOdometry localOdometry;
  DriveTrain drive;

  double horizontalPosition;
  double backPosition;

  public AutoAlignNodes(DriveTrain drive, double horizontalPosition, double backPosition) {
    addRequirements(drive);
    this.drive = drive;
    this.horizontalPosition = horizontalPosition;
    this.backPosition = backPosition;

    localOdometry = new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS, drive.getHeading(), drive.getModulePositions());

    initPIDs();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double[] coords = Limelight.april3DCordsBotPoseTargetSpace();
    Pose2d resetPose = new Pose2d(coords[0], coords[2], drive.getHeading());
    localOdometry.resetPosition(drive.getHeading(), drive.getModulePositions(), resetPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    localOdometry.update(drive.getHeading(), drive.getModulePositions());

    double rotationSpeed = rotationController.calculate(drive.getHeadingDegrees());
    double translateSpeed = translationController.calculate(localOdometry.getPoseMeters().getY());
    double strafeSpeed = strafeController.calculate(localOdometry.getPoseMeters().getX());

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

    translationController = new ProfiledPIDController(1.0, 0, 0, new TrapezoidProfile.Constraints(2.0, 3.0));
    translationController.setGoal(new TrapezoidProfile.State(backPosition, 0.0));
    translationController.setTolerance(0.05);

    strafeController = new ProfiledPIDController(1.0, 0, 0, new TrapezoidProfile.Constraints(2.0, 3.0));
    strafeController.setGoal(new TrapezoidProfile.State(horizontalPosition, 0.0));
    strafeController.setTolerance(0.05);
  }
}
