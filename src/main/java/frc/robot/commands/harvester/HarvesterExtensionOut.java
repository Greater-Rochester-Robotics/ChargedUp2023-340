// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.harvester;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

/**
 * Deploys the harvester.
 */
public class HarvesterExtensionOut extends InstantCommand {
    /**
     * Creates a new HarvesterExtensionOut command.
     */
    public HarvesterExtensionOut () {
        addRequirements(RobotContainer.harvester);
    }

    @Override
    public void initialize () {
        // Deploy the harvester.
        RobotContainer.harvester.harvesterExtensionOut();
    }
}
