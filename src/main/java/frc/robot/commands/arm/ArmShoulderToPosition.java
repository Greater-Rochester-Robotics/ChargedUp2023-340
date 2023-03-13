// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;

/**
 * Moves the shoulder to a set position (in radians).
 */
public class ArmShoulderToPosition extends CommandBase {
    /**
     * The target position to move to.
     */
    private double position;

    /**
     * Creates a new ArmShoulderToPosition command.
     * 
     * @param position The position to set the shoulder to in radians.
     */
    public ArmShoulderToPosition (double position) {
        addRequirements(RobotContainer.arm);
        this.position = position;
    }

    @Override
    public void execute () {
        // Set the shoulder position.
        RobotContainer.arm.setShoulderPosition(position);
    }

    @Override
    public void end (boolean interrupted) {
        // If ended, stop the shoulder from moving.
        RobotContainer.arm.stopShoulder();
    }

    @Override
    public boolean isFinished () {
        // Finish if the shoulder is within tolerance of the target position.
        return Math.abs(RobotContainer.arm.getShoulderPosition() - position) < ArmConstants.SHOULDER_CLOSED_LOOP_ERROR;
    }
}
