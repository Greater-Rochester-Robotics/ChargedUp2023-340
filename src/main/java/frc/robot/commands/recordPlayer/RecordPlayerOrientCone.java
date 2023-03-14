// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.recordPlayer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RecordPlayerConstants;
import frc.robot.RobotContainer;

/**
 * Orients the cone in the record player.
 */
public class RecordPlayerOrientCone extends CommandBase {
    /**
     * The previous state of sensor 0.
     */
    private boolean previousSensor0;
    /**
     * If the cone position has been found.
     */
    private boolean hasFoundPosition = false;
    /**
     * The position to target to orient the cone.
     * Set after the cone's position has been found.
     */
    private double targetPosition;

    /**
     * Creates a new RecordPlayerOrientCone command.
     */
    public RecordPlayerOrientCone () {
        addRequirements(RobotContainer.recordPlayer);
    }

    @Override
    public void initialize () {
        // Start spinning the record player.
        RobotContainer.recordPlayer.setRotationMotor(RecordPlayerConstants.ROTATE_MOTOR_SPEED);

        // Set the previous sensor 0 state.
        previousSensor0 = RobotContainer.recordPlayer.getConePositionSensor0();
    }

    @Override
    public void execute () {
        // Get the current sensor states.
        boolean sensor0 = RobotContainer.recordPlayer.getConePositionSensor0();
        boolean sensor1 = RobotContainer.recordPlayer.getConePositionSensor1();

        // Check to see if the cone position has been found.
        if (!sensor0 && sensor1 && previousSensor0) {
            // If the position was found, set the target position for the record player.
            targetPosition = RobotContainer.recordPlayer.getEncoderAngle() + 0.25;
            hasFoundPosition = true;
        }

        if (hasFoundPosition) {
            // Once the position is found, rotate to the goal position.
            RobotContainer.recordPlayer.rotateToAngle(targetPosition);
        }

        previousSensor0 = sensor0;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end (boolean interrupted) {
        // If ended, stop the record player.
        RobotContainer.recordPlayer.stopRotationMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished () {
        // Stop if the position has been found and the record player is within tolerance of the target position.
        return hasFoundPosition && Math.abs(targetPosition - RobotContainer.recordPlayer.getEncoderAngle()) < 0.1;
    }
}
