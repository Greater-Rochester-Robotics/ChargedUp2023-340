// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Opens the claw and spits.
 */
public class ClawOpenSpit extends SequentialCommandGroup {
    /**
     * Creates a new ClawOpenSpit command.
     */
    public ClawOpenSpit () {
        addCommands(
            new ClawOpen(),
            new ClawSpit()
        );
    }
}
