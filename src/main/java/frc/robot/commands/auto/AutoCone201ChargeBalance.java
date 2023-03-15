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
import frc.robot.commands.drive.DriveBalanceRobot;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.util.DriveSetGyro;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCone201ChargeBalance extends SequentialCommandGroup {
  /** Creates a new AutoCone201ChargeBalance. */
  public AutoCone201ChargeBalance() {
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Cone201ChargeBalance", SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.MAXIMUM_ACCELERATION);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveSetGyro(0),
      new AutoScoreCone(ArmConstants.BACK_MIDDLE_CONE),
      Commands.deadline(
        new DriveFollowTrajectory(path.get(0)),
        new ArmToPosition(ArmConstants.INTERNAL_PICK_UP)
      ),
      new DriveBalanceRobot()
    );
  }
}
