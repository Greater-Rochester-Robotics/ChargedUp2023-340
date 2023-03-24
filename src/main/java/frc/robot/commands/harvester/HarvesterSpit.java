// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.harvester;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

/**
 * Runs the harvester motors to spit a game piece.
 */
public class HarvesterSpit extends InstantCommand {
    /**
     * Creates a new HarvesterSpit command.
     */
    public HarvesterSpit () {
        addRequirements(RobotContainer.harvester);
    }

    @Override
    public void initialize () {
        // Run the harvester motors out.
        RobotContainer.harvester.motorOut();
    }
}
