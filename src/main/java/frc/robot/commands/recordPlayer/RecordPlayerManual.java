// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.recordPlayer;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RecordPlayerConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/**
 * Controls the record player using the co-driver triggers.
 */
public class RecordPlayerManual extends CommandBase {
    /**
     * Creates a new RecordPlayerManual command.
     */
    public RecordPlayerManual () {
        addRequirements(RobotContainer.recordPlayer);
    }

    @Override
    public void execute () {
        // Get the rotation speed.
        double rotationSpeed = (Robot.robotContainer.getCoDriverAxis(Axis.kRightTrigger) - Robot.robotContainer.getCoDriverAxis(Axis.kLeftTrigger)) * RecordPlayerConstants.ROTATE_MOTOR_SPEED;

        // Set the motor speed.
        RobotContainer.recordPlayer.setRotationMotor(rotationSpeed);
    }

    @Override
    public void end (boolean interrupted) {
        // If ended, stop the record player.
        RobotContainer.recordPlayer.stopRotationMotor();
    }

    @Override
    public boolean isFinished () {
        // Run continuously.
        return false;
    }
}
