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
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.HarvestRecordIntake;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.commands.harvester.HarvesterExtensionOut;
import frc.robot.commands.harvester.HarvesterStopRetract;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCone221PickUpReturn extends SequentialCommandGroup {
  /** Creates a new AutoCone221PickUpChargeBalance. */
  public AutoCone221PickUpReturn() {
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Cone221PickUpReturn", SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.MAXIMUM_ACCELERATION);
    
    addCommands(
      new DriveSetGyro(0),
      new AutoScoreCone(ArmConstants.BACK_MIDDLE_CONE),
      new HarvesterExtensionOut(),
      Commands.race(
        new HarvestRecordIntake(false),
        Commands.parallel(
          new ArmToPosition(ArmConstants.INTERNAL_PICK_UP),
          Commands.sequence(
            new DriveFollowTrajectory(path.get(0)),
            new WaitCommand(1.5)
          ),
          Commands.sequence(
            new WaitUntilCommand(RobotContainer.harvester::hasGamePiece),
            new WaitUntilCommand(()-> (!RobotContainer.harvester.hasGamePiece()))
          )
        )
      ),
      new HarvesterStopRetract(false).withTimeout(0),
      new DriveFollowTrajectory(path.get(1),false)

    );
  }
}
