// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

/**
 * Opens the claw.
 */
public class ClawOpen extends InstantCommand {
    /**
     * Creates a new ClawOpen command.
     */
    public ClawOpen () {
        addRequirements(RobotContainer.claw);
    }

    @Override
    public void initialize () {
        // Open the claw.
        RobotContainer.claw.open();
    }
}
