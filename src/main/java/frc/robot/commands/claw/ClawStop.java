// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

/**
 * Stops the claw motors.
 */
public class ClawStop extends InstantCommand {
    /**
     * Creates a new ClawStop command.
     */
    public ClawStop () {
        addRequirements(RobotContainer.claw);
    }

    @Override
    public void initialize () {
        // Stop the claw motors.
        RobotContainer.claw.stop();
    }
}
