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
import frc.robot.commands.HarvesterIntakeCubeWithArm;
import frc.robot.commands.HarvesterRetractWithArm;
import frc.robot.commands.auto.util.AutoScoreCone;
import frc.robot.commands.auto.util.AutoScoreCube;
import frc.robot.commands.drive.DriveBalance;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.util.DriveSetGyro;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCone221PickUpCube212Balance extends SequentialCommandGroup {
  /** Creates a new AutoCone221PickUpCube212Balance. */
  public AutoCone221PickUpCube212Balance() {
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Cone221PickUpCone201Pickup", SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.MAXIMUM_ACCELERATION);
    addCommands(
        new DriveSetGyro(0),
        new AutoScoreCone(ArmConstants.BACK_MIDDLE_CONE),
        new DriveFollowTrajectory(path.get(0)),
        new HarvesterIntakeCubeWithArm(),
        new DriveFollowTrajectory(path.get(1)),
        Commands.parallel(
            new HarvesterRetractWithArm(),
            new DriveFollowTrajectory(path.get(2))
        ),
        new AutoScoreCube(ArmConstants.BACK_HIGH_CUBE),
        new DriveFollowTrajectory(path.get(3)),
        new DriveBalance()

    );
  }
}
