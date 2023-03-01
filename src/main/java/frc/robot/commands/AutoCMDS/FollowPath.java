// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.AutoCMDS;


import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils.Constants.AutoContsants;
import frc.robot.Utils.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

  public class FollowPath {
    PathPlannerTrajectory autoTrajectory = null;
    DriveTrain drive;
    public FollowPath(String path, double maxvel, double maxaccel, DriveTrain drivetrain)
    {
        autoTrajectory = PathPlanner.loadPath(path, maxvel, maxaccel);
       drive = drivetrain;
    }

    public PPSwerveControllerCommand getCommand()
    {
        var thetaController = new PIDController(
        AutoContsants.P_THETA_CONTROLLER, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
        autoTrajectory,
        drive::getPose, // Functional interface to feed supplier
        DriveConstants.DRIVE_KINEMATICS,

        // Position controllers
        new PIDController(AutoContsants.PX_CONTROLLER, 0, 0),
        new PIDController(AutoContsants.PY_CONTROLLER, 0, 0),
        thetaController,
       drive::setModuleStates,
        drive);

    // Reset odometry to the starting pose of the trajectory.
    //m_robotDrive.resetOdometry(autoTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand;
    }
}
