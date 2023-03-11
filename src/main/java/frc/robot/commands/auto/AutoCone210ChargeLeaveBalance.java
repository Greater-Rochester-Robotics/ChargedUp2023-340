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
import frc.robot.commands.ArmClawStopRetract;
import frc.robot.commands.ArmScoreCone;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.drive.DriveBalanceRobot;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.subsystems.ArmPosition;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCone210ChargeLeaveBalance extends SequentialCommandGroup {
  /** Creates a new AutoCone20ChargeBalance. */
  public AutoCone210ChargeLeaveBalance() {
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Cone210ChargeLeaveBalance", SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.MAXIMUM_ACCELERATION);

    addCommands(
      new DriveSetGyro(0),
      new AutoScoreCone(ArmConstants.BACK_MIDDLE_CONE),
      Commands.parallel(
        new ArmToPosition(ArmConstants.INTERNAL_PICK_UP),
        new DriveFollowTrajectory(path.get(0))
      ),
      Commands.race(
        new DriveBalanceRobot(),
        new WaitUntilCommand(()->(Math.abs( RobotContainer.swerveDrive.getGyroInDegPitch()) < SwerveDriveConstants.DRIVE_BALANCE_ROBOT_ANGLE_TOLERANCE))
      ),
      new DriveFollowTrajectory(path.get(1),true),
      new DriveBalanceRobot()      
    );
  }
}
