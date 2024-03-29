// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

/**
 * Closes the claw.
 */
public class ClawClose extends InstantCommand {
    private boolean isCone;
    /**
     * Creates a new ClawClose command.
     */
    public ClawClose (boolean isCone) {
        addRequirements(RobotContainer.claw);
        this.isCone = isCone;
    }

    @Override
    public void initialize () {
        // Close the claw.
        RobotContainer.claw.close(isCone);
    }
}
