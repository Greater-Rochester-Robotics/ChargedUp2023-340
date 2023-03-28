/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveDriveConstants;

/**
 * This command is designed so that a driver can drive
 * the swerve drive based around the robot's orientation.
 * Forward on the stick will cause the robot to drive
 * forward. left and right on the stick will cause the
 * robot to move to its left or right. This command does
 * not end of its own accord so it must be interupted to
 * end.
 */
public class DriveRobotCentric extends CommandBase {
    // isVeloMode says if velosity mode is inabled
    private boolean isVeloMode;

    /**
     * Creates a new DriveRobotCentric.
     */
    public DriveRobotCentric (boolean isVeloMode) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.swerveDrive);
        this.isVeloMode = isVeloMode;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize () {
        // RobotContainer.swerveDrive.setIsOdometry(false);
        RobotContainer.setDriverRumble(0.25, 0.25);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute () {
        RobotContainer.setDriverRumble(0.25, 0.25);
        double forwardSpeed = Robot.robotContainer.getRobotForwardFull(isVeloMode);
        double strafeSpeed = Robot.robotContainer.getRobotLateralFull(isVeloMode);
        double rotSpeed = Robot.robotContainer.getRobotRotation(isVeloMode);

        //check if secondary sticks are being used
        if(Robot.robotContainer.getDriverButton(9)){
          //if secondary sticks used, replace with secondary sticks witha slow factor
          forwardSpeed *= SwerveDriveConstants.DRIVER_SLOW_STICK_ROT_MODIFIER;
          strafeSpeed *= SwerveDriveConstants.DRIVER_SLOW_STICK_ROT_MODIFIER;
          rotSpeed *= SwerveDriveConstants.DRIVER_SLOW_STICK_ROT_MODIFIER;
        }

        RobotContainer.swerveDrive.driveRobotCentric(
            forwardSpeed,
            strafeSpeed,
            rotSpeed,
            isVeloMode,
            false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end (boolean interrupted) {
        RobotContainer.setDriverRumble(0.0, 0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished () {
        return false;
    }
}
