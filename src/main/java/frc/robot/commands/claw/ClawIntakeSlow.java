// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

/**
 * Slowly runs the claw motors. Used for holding game pieces.
 */
public class ClawIntakeSlow extends InstantCommand {
    /**
     * Creates a new ClawIntakeSlow command.
     */
    public ClawIntakeSlow () {
        addRequirements(RobotContainer.claw);
    }

    @Override
    public void initialize () {
        // Run the claw motors in slowly.
        RobotContainer.claw.intakeSlow();
    }
}
