// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class ExampleAutoTwo implements AutoInterface {
    private String pathString = "pathplanner/generatedJSON/Test2.wpilib.json";
    private Trajectory autoTrajectory = null;
    public ExampleAutoTwo()
    {
        try{
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(pathString);
            autoTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
          }
          catch (IOException ex){
            DriverStation.reportWarning("Could not load " + pathString, ex.getStackTrace());
          }
    }
    public Command getAutoCommand()
    {
        var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        autoTrajectory,
        RobotContainer.m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        RobotContainer.m_robotDrive::setModuleStates,
        RobotContainer.m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        RobotContainer.m_robotDrive.resetOdometry(autoTrajectory.getInitialPose());

        return swerveControllerCommand;
    }
}
