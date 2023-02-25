// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;

/** Add your docs here. */
public class FollowPath extends CommandBase {
    Timer timer = new Timer();
    PathPlannerTrajectory trajectory;
    HolonomicDriveController controller;
    boolean resetOdometry;

    String pathName;

    public FollowPath(String pathname) {
        this(pathname, AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared, true);
    }

    public FollowPath(String pathname, double maxVel, double maxAccel) {
        this(pathname, maxVel, maxAccel, true);
    }

    public FollowPath(String pathName, double maxVel, double maxAccel, boolean resetOdometry) {
        addRequirements(RobotContainer.m_robotDrive);

        this.trajectory = PathPlanner.loadPath(pathName, maxVel, maxAccel);

        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.controller = new HolonomicDriveController(xController, yController, thetaController);
        this.resetOdometry = resetOdometry;

        this.pathName = pathName;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        Pose2d initialPose = trajectory.getInitialPose();
        if(resetOdometry) RobotContainer.m_robotDrive.resetOdometry(new Pose2d(initialPose.getTranslation(), RobotContainer.m_robotDrive.getRotation2d()));
    }

    @Override
    public void execute(){
        double time = timer.get();
        PathPlannerState desiredState = (PathPlannerState) trajectory.sample(time);
        ChassisSpeeds targetSpeeds = controller.calculate(RobotContainer.m_robotDrive.getPose(), desiredState, new Rotation2d(desiredState.holonomicRotation.getRadians()));

        targetSpeeds.vyMetersPerSecond = -targetSpeeds.vyMetersPerSecond;
        targetSpeeds.omegaRadiansPerSecond = -targetSpeeds.omegaRadiansPerSecond;

        // System.out.println("x:   " + RobotContainer.drive.getPoseMeters().getTranslation().getX() + " y:   " + RobotContainer.drive.getPoseMeters().getTranslation().getY() + " r: " + RobotContainer.drive.getPoseMeters().getRotation().getDegrees());
        // System.out.println("tx:  " + desiredState.poseMeters.getTranslation().getX() + " ty: " + desiredState.poseMeters.getTranslation().getY() + " tr:" + desiredState.holonomicRotation.getDegrees());
        // System.out.println("tvx: " + targetSpeeds.vxMetersPerSecond + " tvy: " + targetSpeeds.vyMetersPerSecond);
        // Position PID
        // SmartDashboard.putNumber("PIDTarget", 0);
        // SmartDashboard.putNumber("PIDActual", pathController.getPosError());

        // Rotation PID
        // SmartDashboard.putNumber("PIDTarget", desiredState.holonomicRotation.getDegrees());
        // SmartDashboard.putNumber("PIDACtual", RobotContainer.drive.getAngleDegrees());

        // Heading PID
        // SmartDashboard.putNumber("PIDTarget", desiredState.poseMeters.getRotation().getDegrees());
        // SmartDashboard.putNumber("PIDActual", pathController.getCurrentHeading().getDegrees());
        // System.out.println("tr:" + Math.round(desiredState.holonomicRotation.getDegrees()) + ", " + "r:" + Math.round(RobotContainer.drive.getAngleDegrees()) + " | th:" + Math.round(desiredState.poseMeters.getRotation().getDegrees()));

        Pose2d currentPose = RobotContainer.m_robotDrive.getPose();
        String tString = " [" + Math.round(timer.get() * 100) / 100.0 + "]";
        System.out.println(pathName + tString + " x error: " + (desiredState.poseMeters.getX() - currentPose.getX()));
        System.out.println(pathName + tString + " y error: " + (desiredState.poseMeters.getY() - currentPose.getY()));
        System.out.println(pathName + tString + " r error: " + (desiredState.holonomicRotation.getDegrees() - currentPose.getRotation().getDegrees()));

        RobotContainer.m_robotDrive.drive(targetSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        RobotContainer.m_robotDrive.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}
