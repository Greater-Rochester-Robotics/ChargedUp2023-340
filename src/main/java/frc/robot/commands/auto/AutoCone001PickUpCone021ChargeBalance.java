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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.ClawWristExtend;
import frc.robot.commands.ClawWristRetract;
import frc.robot.commands.HarvesterClawIntake;
import frc.robot.commands.HarvesterRecordRetract;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.auto.util.AutoDriveFollowTrajectory;
import frc.robot.commands.auto.util.AutoScoreCone;
import frc.robot.commands.drive.DriveBalanceRobot;
import frc.robot.commands.drive.util.DriveSetGyro;
import frc.robot.commands.harvester.HarvesterExtensionOut;
import frc.robot.subsystems.swervelib.ADIS16470_IMU.IMUAxis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCone001PickUpCone021ChargeBalance extends SequentialCommandGroup {
  /** Creates a new Cone221PickUpCone201ChargeBalance. */
  public AutoCone001PickUpCone021ChargeBalance() {
    // Load path group
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Cone001PickUpCone021ChargeBalance", SwerveDriveConstants.PATH_MAXIMUM_VELOCITY, SwerveDriveConstants.MAXIMUM_ACCELERATION);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Zero gyro on all angles so pathing and balancing works
        new DriveSetGyro(0),
        new DriveSetGyro(0, IMUAxis.kPitch),
        new DriveSetGyro(0, IMUAxis.kRoll),

        // Score cone mid
        new AutoScoreCone(ArmConstants.BACK_MIDDLE_CONE).withTimeout(6.0),


        Commands.parallel(
            // Drive to pick-up location
            Commands.sequence(
                new AutoDriveFollowTrajectory(path.get(0)),
                new WaitCommand(.5)
            ),

            Commands.sequence(
                new ArmToPosition(ArmConstants.INTERNAL_DEFAULT),

                // Put out intake
                new HarvesterClawIntake(true)
            )

            // Wait for cone to be intook (Wait to see cone then wait to not see anymore)
            // Commands.sequence(
            //     new WaitUntilCommand(RobotContainer.harvester::hasGamePiece).withTimeout(1.5),
            //     new WaitUntilCommand(() -> (!RobotContainer.harvester.hasGamePiece())).withTimeout(0.5)
            // )
        ),

        Commands.parallel(
            Commands.sequence(
                // Drive to second scoring position
                new AutoDriveFollowTrajectory(path.get(1), false)
            ),

            Commands.sequence(
                // Bring in intake an orient cone with spindexer
                new HarvesterRecordRetract(true, 0.25, 0.5),

                // Move arm to pick-up position for cone
                new ArmToPosition(ArmConstants.INTERNAL_PICK_UP_CONE),
    
                // Extend wrist to cone
                new ClawWristExtend(),

                // Grab cone and retract wrist
                new ClawWristRetract(true)
            )
        ),
        
        // Score cone mid
        new AutoScoreCone(ArmConstants.BACK_MIDDLE_CONE).withTimeout(6.0),

        // Drive to balancing position bring arm to default position
        Commands.deadline(
            new AutoDriveFollowTrajectory(path.get(2)),
            new ArmToPosition(ArmConstants.INTERNAL_PICK_UP_CONE)
        ).withTimeout(6),

        // Balance robot with feedback loop
        new DriveBalanceRobot()
    );
  }
}
