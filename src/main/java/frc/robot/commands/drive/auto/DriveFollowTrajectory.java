// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * Drives the robot following a trajectory generated by PathPlanner.
 */
public class DriveFollowTrajectory extends CommandBase {
    /**
     * Timer used when running the path.
     */
    private Timer timer = new Timer();
    /**
     * The trajectory, generated by PathPlanner.
     */
    private PathPlannerTrajectory trajectory;
    /**
     * If this command resets odometry to the starting point.
     */
    boolean resetOdometry;

    /**
     * Run the path with default maximum velocity and acceleration.
     * 
     * @param pathFileName Name of the file generated by PathPlanner.
     */
    public DriveFollowTrajectory (String pathFileName) {
        this(pathFileName, Constants.SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, Constants.SwerveDriveConstants.MAXIMUM_ACCELERATION);
    }

    /**
     * Run the path.
     * 
     * @param pathFileName Name of the file generated by PathPlanner.
     * @param maxVel The maximum allowed velocity.
     * @param maxAccel The maximum allowed acceleration.
     */
    public DriveFollowTrajectory (String pathFileName, double maxVel, double maxAccel) {
        this(pathFileName, maxVel, maxAccel, true);
    }

    /**
     * Run the path.
     * 
     * @param pathFileName Name of the file generated by PathPlanner.
     * @param maxVel The maximum allowed velocity.
     * @param maxAccel The maximum allowed acceleration.
     * @param resetOdometry If odometry should be reset to the starting point.
     */
    public DriveFollowTrajectory (String pathFileName, double maxVel, double maxAccel, boolean resetOdometry) {
        this(PathPlanner.loadPath(pathFileName, maxVel, maxAccel), resetOdometry);
    }

    /**
     * Run a specified path, with odometry being reset to the starting point.
     * 
     * @param trajectory The trajectory to run.
     */
    public DriveFollowTrajectory (PathPlannerTrajectory trajectory) {
        this(trajectory, true);
    }

    /**
     * Run a specified path.
     * 
     * @param trajectory The trajectory to run.
     * @param resetOdometry If odometry should be reset to the starting point.
     */
    public DriveFollowTrajectory (PathPlannerTrajectory trajectory, boolean resetOdometry) {
        addRequirements(RobotContainer.swerveDrive);
        this.trajectory = trajectory;
        this.resetOdometry = resetOdometry;
    }

    @Override
    public void initialize () {
        // Start the timer.
        timer.reset();
        timer.start();

        // Modify the trajectory to work with the robot's alliance.
        trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());

        // Find the first point in the trajectory,
        PathPlannerState initialState = trajectory.getInitialState();

        // Reset the PID controller.
        RobotContainer.swerveDrive.resetTrajectoryPIDControllers();

        // Reset odometry if specified.
        if (resetOdometry) {
            RobotContainer.swerveDrive.setCurPose2d(new Pose2d(initialState.poseMeters.getTranslation(), RobotContainer.swerveDrive.getGyroRotation2d()));
        }
    }

    @Override
    public void execute () {
        // Find the robot's target location on the trajectory based on the current time.
        PathPlannerState desiredState = (PathPlannerState) trajectory.sample(timer.get());

        // Adjust the robot's speed based on the target.
        ChassisSpeeds robotSpeed = RobotContainer.swerveDrive.calculateSpeedsTraj(desiredState);

        // Pass the speed to the swerve subsystem.
        RobotContainer.swerveDrive.driveRobotCentric(robotSpeed, true, false);
    }

    @Override
    public void end (boolean interrupted) {
        // Stop the timer.
        timer.stop();

        // Stop the robot.
        RobotContainer.swerveDrive.driveRobotCentric(0, 0, 0, true, false);
    }

    @Override
    public boolean isFinished () {
        // Finish after the timer reaches the time the trajectory takes to run.
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}