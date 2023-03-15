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
import frc.robot.commands.HarvesterRecordRetract;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.auto.util.AutoDriveFollowTrajectory;
import frc.robot.commands.auto.util.AutoScoreCone;
import frc.robot.commands.drive.DriveBalanceRobot;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.commands.harvester.HarvesterIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCone220PickupCone221ChargeBalance extends SequentialCommandGroup {
  /** Creates a new AutoCone220PickupCone221ChargeBalance. */
  public AutoCone220PickupCone221ChargeBalance() {
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Cone220PickupCone221ChargeBalance", SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.MAXIMUM_ACCELERATION);

    addCommands(
        new DriveSetGyro(0),
        new AutoScoreCone(ArmConstants.BACK_HIGH_CONE),
        Commands.race(
            new HarvesterIntake(false),
            Commands.parallel(
            new ArmToPosition(ArmConstants.INTERNAL_PICK_UP),
            Commands.sequence(
                new AutoDriveFollowTrajectory(path.get(0)),
                new WaitCommand(0.5)
            ),
            Commands.sequence(
                new WaitUntilCommand(RobotContainer.harvester::hasGamePiece),
                new WaitUntilCommand(()-> (!RobotContainer.harvester.hasGamePiece()))
            )
            )
        ),
        new AutoDriveFollowTrajectory(path.get(1),false),
        new WaitCommand(0.5),
        new HarvesterRecordRetract(false),
        new AutoScoreCone(ArmConstants.BACK_HIGH_CONE),
        new AutoDriveFollowTrajectory(path.get(2),false),
        new DriveBalanceRobot()
    );
  }
}
