// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;


public class DriveToTarget extends InstantCommand {

  PathPlannerTrajectory trajectory;
  /** 
   * THIS IS NOT FOR AUTO'S. It this command spawns another 
   * command that can kill command groups.
   */
  public DriveToTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Translation2d goalTranslation = RobotContainer.target.getTargetPosition().getPosition();
    Rotation2d goalRotation = new Rotation2d(0);//TODO: fix for scoring out the front

    Pose2d startPosition = RobotContainer.swerveDrive.getCurPose2d();
    // System.out.println("Start pose: " + startPosition + "\tGoal pose: " + goalTranslation);

    PathConstraints constraints = new PathConstraints(SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.MAXIMUM_ACCELERATION);
    List<EventMarker> eventMarkers = new ArrayList<EventMarker>();

    PathPoint goalPose = new PathPoint(goalTranslation, goalRotation, 0);
    PathPoint startPose = new PathPoint(startPosition.getTranslation(), startPosition.getRotation());
    
    trajectory = PathPlanner.generatePath(constraints, eventMarkers, startPose, goalPose);

    (new DriveFollowTrajectory(trajectory)).schedule();

    
  }

  
}
