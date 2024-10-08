// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.HarvesterIntakeCubeWithArm;
import frc.robot.commands.HarvesterRetractCubeWithArm;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.auto.util.AutoScoreCone;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawSpit;
import frc.robot.commands.claw.ClawStop;
import frc.robot.commands.drive.DriveBalance;
import frc.robot.commands.drive.auto.DriveFollowTrajectory;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.subsystems.swervelib.ADIS16470_IMU.IMUAxis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCone221PickUpCube211PickUpCube extends SequentialCommandGroup {
  /** Creates a new Cone221PickUpCone201ChargeBalance. */
   /**
    * @param scoreConeHigh Whether to score the cone high, scores mid on false
    * @param scoreCubeHigh Whether to score the cone high, scores mid on false
    */
  public AutoCone221PickUpCube211PickUpCube(boolean scoreConeHigh, boolean scoreCubeHigh) {
    // Load path group
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Cone221PickUpCube211PickUpCube", 
        new PathConstraints(SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.MAXIMUM_ACCELERATION), 
        new PathConstraints(SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.MAXIMUM_ACCELERATION), 
        new PathConstraints(SwerveDriveConstants.PATH_MAXIMUM_VELOCITY - 1.0, SwerveDriveConstants.MAXIMUM_ACCELERATION));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Zero gyro on all angles so pathing and balancing works
        new DriveSetGyro(0),
        new DriveSetGyro(0, IMUAxis.kPitch),
        new DriveSetGyro(0, IMUAxis.kRoll),

        // Score cone mid
        new AutoScoreCone(scoreConeHigh?ArmConstants.BACK_HIGH_CONE:ArmConstants.BACK_MIDDLE_CONE).withTimeout(6.0),


        Commands.parallel(
            // Drive to pick-up location
            Commands.sequence(
                new DriveFollowTrajectory(path.get(0)),
                new WaitCommand(0.5)
            ),

            new HarvesterIntakeCubeWithArm()
        ),

        Commands.parallel(
            // Drive to second scoring position
            new DriveFollowTrajectory(path.get(1), false),

            Commands.sequence(
                new WaitCommand(.25),
                // Grab cone and retract wrist. Move the arm to position
                new HarvesterRetractCubeWithArm(scoreCubeHigh?ArmConstants.BACK_HIGH_CUBE:ArmConstants.BACK_MIDDLE_CUBE),

                // Score the cube
                new ClawSpit()
            ).withTimeout(7)
        ),
        
        //TODO: Double Check
        Commands.parallel(
            // Drive to pick-up location
            Commands.sequence(
                new DriveFollowTrajectory(path.get(2)),
                new WaitCommand(0.5)
            ),

            new HarvesterIntakeCubeWithArm()
        ),
        new ParallelCommandGroup(
            new DriveFollowTrajectory(path.get(3)),
            new HarvesterRetractCubeWithArm()
        )
    );
  }
}
