// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.HarvestRecordIntake;
import frc.robot.commands.HarvesterRecordRetract;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.auto.util.AutoDriveFollowTrajectory;
import frc.robot.commands.auto.util.AutoScoreCone;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.commands.harvester.HarvesterExtensionOut;

public class AutoCone001PickUpReturn extends SequentialCommandGroup {
    public AutoCone001PickUpReturn () {
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Cone001PickUpReturn", SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.MAXIMUM_ACCELERATION);

        addCommands(
            new DriveSetGyro(0),
            new AutoScoreCone(ArmConstants.BACK_MIDDLE_CONE),
            new HarvesterExtensionOut(),
            Commands.parallel(
                new HarvestRecordIntake(false),
                new ArmToPosition(ArmConstants.INTERNAL_PICK_UP),
                Commands.sequence(
                    new AutoDriveFollowTrajectory(path.get(0)),
                    new WaitCommand(1.5)
                ),
                Commands.sequence(
                    new WaitUntilCommand(RobotContainer.harvester::hasGamePiece),
                    new WaitUntilCommand(() -> (!RobotContainer.harvester.hasGamePiece()))
                )
            ),
            new HarvesterRecordRetract(false).withTimeout(0),
            new AutoDriveFollowTrajectory(path.get(1), false)
        );
    }
}
