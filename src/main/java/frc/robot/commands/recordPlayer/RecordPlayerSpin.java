// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.recordPlayer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.RecordPlayerConstants;
import frc.robot.RobotContainer;

/**
 * Spins the record player.
 */
public class RecordPlayerSpin extends InstantCommand {
    /**
     * The speed to run the record player at.
     */
    private double speed;

    /**
     * Creates a new RecordPlayerSpin command.
     */
    public RecordPlayerSpin () {
        this(RecordPlayerConstants.ROTATE_MOTOR_SPEED);
    }

    /**
     * Creates a new RecordPlayerSpin command.
     * 
     * @param speed The speed to run the record player at.
     */
    public RecordPlayerSpin (double speed) {
        addRequirements(RobotContainer.recordPlayer);
        this.speed = speed;
    }

    @Override
    public void initialize () {
        // Spin the record player.
        RobotContainer.recordPlayer.setRotationMotor(speed);
    }
}
