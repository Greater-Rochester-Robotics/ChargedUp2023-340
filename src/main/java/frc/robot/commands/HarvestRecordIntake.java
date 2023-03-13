// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.harvester.HarvesterExtensionOut;
import frc.robot.commands.harvester.HarvesterIntake;
import frc.robot.commands.recordPlayer.RecordPlayerSpin;

/**
 * Deploys the harvester and runs the harvester motors.
 */
public class HarvestRecordIntake extends SequentialCommandGroup {
    /**
     * Creates a new HarvestRecordIntake command.
     * 
     * @param isCone If the harvester is grabbing a cone.
     */
    public HarvestRecordIntake (boolean isCone) {
        addCommands(
            // Deploy the harvester.
            new HarvesterExtensionOut(),

            // Run the intake motors.
            new HarvesterIntake(isCone),

            // If picking up a cone, close the claw to utilize the flanges to knock over
            // upright cones in the record player, and spin the record player.
            new ConditionalCommand(
                Commands.sequence(
                    new RecordPlayerSpin(),
                    new ClawClose()
                ),
                new InstantCommand(),
                () -> isCone
            )
        );
    }
}
