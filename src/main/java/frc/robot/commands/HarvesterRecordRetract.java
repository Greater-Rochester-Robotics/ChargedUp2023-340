// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.harvester.HarvesterExtensionIn;
import frc.robot.commands.harvester.HarvesterStop;
import frc.robot.commands.recordPlayer.RecordPlayerOrientCone;
import frc.robot.commands.recordPlayer.RecordPlayerSpin;

/**
 * Stops the harvester motors and retracts the harvester into the robot.
 */
public class HarvesterRecordRetract extends SequentialCommandGroup {
    /**
     * Creates a new HarvesterRecordRetract command.
     */
    public HarvesterRecordRetract (boolean isCone) {
        addCommands(
            // Bring in the harvester.
            new HarvesterExtensionIn(),

            // Add delay to ensure the game piece is in the record player.
            new WaitCommand(1),

            // Stop the harvester motors.
            new HarvesterStop(),

            new WaitCommand(2.5),

            // If we grabbed a cone, add delay to ensure the cone is in position then run the record player.
            new ConditionalCommand(new RecordPlayerOrientCone(), new InstantCommand(), () -> isCone)
        );
    }
}
