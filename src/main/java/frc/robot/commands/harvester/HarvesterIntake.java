// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.harvester;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

/**
 * Runs the harvester motors to intake a game piece.
 */
public class HarvesterIntake extends InstantCommand {
    /**
     * If the harvester is grabbing a cone.
     */
    boolean isCone;

    /**
     * Creates a new HarvesterIntake command.
     * 
     * @param isCone If the harvester is grabbing a cone.
     */
    public HarvesterIntake (boolean isCone) {
        addRequirements(RobotContainer.harvester);
        this.isCone = isCone;
    }

    @Override
    public void initialize () {
        // Run the harvester motors in.
        RobotContainer.harvester.motorIn(isCone);
    }
}
