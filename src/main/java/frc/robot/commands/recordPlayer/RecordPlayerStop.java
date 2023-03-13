// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.recordPlayer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

/**
 * Stops the record player.
 */
public class RecordPlayerStop extends InstantCommand {
    /**
     * Creates a new RecordPlayerStop command.
     */
    public RecordPlayerStop () {
        addRequirements(RobotContainer.recordPlayer);
    }

    @Override
    public void initialize () {
        // Stop the record player.
        RobotContainer.recordPlayer.stopRotationMotor();
    }
}
