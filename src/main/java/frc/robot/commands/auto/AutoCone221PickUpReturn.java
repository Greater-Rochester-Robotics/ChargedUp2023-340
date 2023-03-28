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
import frc.robot.commands.HarvesterClawIntake;
import frc.robot.commands.HarvesterIntakeCubeWithArm;
import frc.robot.commands.HarvesterRecordRetract;
import frc.robot.commands.HarvesterRetractCubeWithArm;
import frc.robot.commands.auto.util.AutoScoreCone;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.commands.harvester.HarvesterExtensionOut;

public class AutoCone221PickUpReturn extends SequentialCommandGroup {
    /**
     * @param scoreHigh Whether to score the cone high, scores mid on false
     */
    public AutoCone221PickUpReturn (boolean scoreHigh) {
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Cone221PickUpReturn", SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.MAXIMUM_ACCELERATION);

        addCommands(
            new DriveSetGyro(0),
            new AutoScoreCone(scoreHigh?ArmConstants.BACK_HIGH_CONE:ArmConstants.BACK_MIDDLE_CONE).withTimeout(6.0),
            Commands.parallel(
                // Drive to pick-up location
                Commands.sequence(
                    new DriveFollowTrajectory(path.get(0)),
                    new WaitCommand(0.5)
                ),

                new HarvesterIntakeCubeWithArm()
            ),

            Commands.parallel(
                Commands.sequence(
                    // Drive to second scoring position
                    new DriveFollowTrajectory(path.get(1), false)
                ),

                Commands.sequence(
                    new WaitCommand(.25),
                    // Grab cone and retract wrist. Move the arm to position
                    new HarvesterRetractCubeWithArm()
                )
            )
        );
    }
}
