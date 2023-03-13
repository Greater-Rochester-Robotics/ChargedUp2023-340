// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;

/**
 * Moves the elbow to a set position (in radians).
 */
public class ArmElbowToPosition extends CommandBase {
    /**
     * If the elbow is going down.
     */
    private boolean goingDown;
    /**
     * A counter for the number of times the target position has been hit to combat slop.
     * If this value exceeds 10, the elbow is assumed to be at position.
     * Incremented if the arm is within tolerance when execute() is called.
     */
    private int hitTarget;
    /**
     * The target position to move to.
     */
    private double position;

    /**
     * Creates a new ArmElbowToPosition command.
     * 
     * @param position The position to set the elbow to in radians.
     */
    public ArmElbowToPosition (double position) {
        addRequirements(RobotContainer.arm);
        this.position = position;
    }

    @Override
    public void initialize() {
        // Set helpers.
        goingDown = Math.abs(RobotContainer.arm.getElbowPosition()) > Math.abs(position);
        hitTarget = 0;
    }

    @Override
    public void execute () {
        // Set the elbow position.
        RobotContainer.arm.setElbowPosition(position);

        // If the elbow is within tolerance of the target position, increment hitTarget. Otherwise, reset the hitTarget count.
        if (Math.abs(RobotContainer.arm.getElbowPosition() - position) < ArmConstants.ELBOW_CLOSED_LOOP_ERROR) {
            hitTarget++;
        } else {
            hitTarget = 0;
        }
    }

    @Override
    public void end (boolean interrupted) {
        // If ended, stop the elbow from moving.
        RobotContainer.arm.stopElbow();
    }

    @Override
    public boolean isFinished () {
        // Finish if hitTarget has been incremented to 10, or if the elbow has exceeded the maximum safe angle.
        return hitTarget >= 10 || (!goingDown && Math.abs(RobotContainer.arm.getElbowPosition()) > ArmConstants.MAX_ELBOW_ANGLE);
    }
}
