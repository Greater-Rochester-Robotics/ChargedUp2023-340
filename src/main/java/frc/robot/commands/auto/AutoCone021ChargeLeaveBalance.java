// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.auto.util.AutoDriveFollowTrajectory;
import frc.robot.commands.auto.util.AutoScoreCone;
import frc.robot.commands.drive.DriveBalanceRobot;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.subsystems.swervelib.ADIS16470_IMU.IMUAxis;

public class AutoCone021ChargeLeaveBalance extends SequentialCommandGroup {
    public AutoCone021ChargeLeaveBalance () {
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Cone021ChargeLeaveBalance", SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.MAXIMUM_ACCELERATION);

    addCommands(
      new DriveSetGyro(0),
      new DriveSetGyro(0, IMUAxis.kPitch),
      new DriveSetGyro(0, IMUAxis.kRoll),
      new AutoScoreCone(ArmConstants.BACK_MIDDLE_CONE),
      Commands.deadline(
        new AutoDriveFollowTrajectory(path.get(0)),
        new ArmToPosition(ArmConstants.INTERNAL_PICK_UP_CONE)
      ).withTimeout(6),
      new DriveSetGyro(0, IMUAxis.kPitch),
      new DriveSetGyro(0, IMUAxis.kRoll),
      Commands.race(
        new DriveBalanceRobot(),
        new WaitUntilCommand(()->(Math.abs( RobotContainer.swerveDrive.getGyroInDegPitch()) < SwerveDriveConstants.DRIVE_BALANCE_ROBOT_ANGLE_TOLERANCE_AUTO))
      ),
      new AutoDriveFollowTrajectory(path.get(1),true),
      new DriveSetGyro(0, IMUAxis.kPitch),
      new DriveSetGyro(0, IMUAxis.kRoll),
      new DriveBalanceRobot()
    );
  }
}
