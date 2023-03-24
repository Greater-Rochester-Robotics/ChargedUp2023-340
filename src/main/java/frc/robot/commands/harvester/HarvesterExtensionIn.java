// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.harvester;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

/**
 * Retracts the harvester into the robot.
 */
public class HarvesterExtensionIn extends InstantCommand {
    /**
     * Creates a new HarvesterExtensionIn command.
     */
    public HarvesterExtensionIn () {
        addRequirements(RobotContainer.harvester);
    }

    @Override
    public void initialize () {
        // Retract the harvester into the robot.
        RobotContainer.harvester.harvesterExtensionIn();
    }
}
