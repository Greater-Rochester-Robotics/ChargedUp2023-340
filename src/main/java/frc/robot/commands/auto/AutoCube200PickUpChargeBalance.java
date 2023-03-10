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
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.HarvestRecordIntake;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.drive.DriveBalanceRobot;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.commands.harvester.HarvesterIntake;
import frc.robot.commands.harvester.HarvesterStopRetract;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCube200PickUpChargeBalance extends SequentialCommandGroup {
  /** Creates a new AutoCone20PickUpChargeBalance. */
  public AutoCube200PickUpChargeBalance() {
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("AutoCone20PickUpChargeBalance", SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.MAXIMUM_ACCELERATION);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveSetGyro(0),
      new ArmToPosition(ArmConstants.BACK_HIGH_CUBE),
      new ClawOpen(),
      new WaitCommand(0.5),
      Commands.race(
        new HarvesterIntake(false),
        Commands.parallel(
          new ArmToPosition(ArmConstants.INTERNAL_PICK_UP),
          Commands.sequence(new DriveFollowTrajectory(path.get(0))),
            new WaitCommand(0.5)
          ),
          Commands.sequence(
            new WaitUntilCommand(RobotContainer.harvester::hasGamePiece),
            new WaitUntilCommand(()-> (!RobotContainer.harvester.hasGamePiece()))
        ),
      new DriveFollowTrajectory(path.get(1),false),
      new WaitCommand(0.5),
      Commands.parallel(
        new HarvesterStopRetract(false),
        new DriveFollowTrajectory(path.get(2),false)
        )
      ),
      new DriveBalanceRobot()
    );
  }
}
