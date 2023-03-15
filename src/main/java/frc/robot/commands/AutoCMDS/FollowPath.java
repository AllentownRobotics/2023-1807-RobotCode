// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCMDS;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.Utils.Constants.AutoContsants;
import frc.robot.Utils.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class FollowPath {
    PathPlannerTrajectory autoTrajectory = null;
    DriveTrain m_robotDrive;
    SwerveDriveOdometry odometry;

    public FollowPath(String path, double maxvel, double maxaccel, DriveTrain driveSubsystem)
    {
        autoTrajectory = PathPlanner.loadPath(path, maxvel, maxaccel);
        m_robotDrive = driveSubsystem;
        odometry = driveSubsystem.getOdometryInstance();
    }

    public FollowPath(PathPlannerTrajectory path, double maxVelocity, double maxAccel, DriveTrain driveSubsystem, SwerveDriveOdometry odometry){
        autoTrajectory = path;
        m_robotDrive = driveSubsystem;
        this.odometry = odometry;
    }

    public PPSwerveControllerCommand getCommand()
    {
        PIDController thetaController = new PIDController(AutoContsants.P_THETA_CONTROLLER, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
            autoTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.DRIVE_KINEMATICS,

        // Position controllers
            new PIDController(AutoContsants.PX_CONTROLLER, 0, 0),
            new PIDController(AutoContsants.PY_CONTROLLER, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            false,
            m_robotDrive);

    // Run path following command, then stop at the end.
    return swerveControllerCommand;
    }
}
