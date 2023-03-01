// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class FollowPath {
    PathPlannerTrajectory autoTrajectory = null;
    DriveSubsystem m_robotDrive;
    public FollowPath(String path, double maxvel, double maxaccel, DriveSubsystem driveSubsystem)
    {
        autoTrajectory = PathPlanner.loadPath(path, maxvel, maxaccel);
        m_robotDrive = driveSubsystem;
    }

    public PPSwerveControllerCommand getCommand()
    {
        var thetaController = new PIDController(
        AutoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
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
    //m_robotDrive.resetOdometry(autoTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand;
    }
}
