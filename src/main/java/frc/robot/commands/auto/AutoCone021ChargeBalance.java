// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.auto.util.AutoDriveFollowTrajectory;
import frc.robot.commands.auto.util.AutoScoreCone;
import frc.robot.commands.drive.DriveBalance;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.subsystems.swervelib.ADIS16470_IMU.IMUAxis;

public class AutoCone021ChargeBalance extends SequentialCommandGroup {
  /** Creates a new AutoCone021ChargeBalance. */
  public AutoCone021ChargeBalance() {
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Cone021ChargeBalance", SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.MAXIMUM_ACCELERATION);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveSetGyro(0),
      new DriveSetGyro(0, IMUAxis.kPitch),
      new DriveSetGyro(0, IMUAxis.kRoll),
      new AutoScoreCone(ArmConstants.BACK_MIDDLE_CONE).withTimeout(6),
      Commands.deadline(
        new AutoDriveFollowTrajectory(path.get(0)),
        new ArmToPosition(ArmConstants.INTERNAL_PICK_UP_CONE)
      ).withTimeout(6),
      new DriveBalance()
    );
  }
}
