// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.harvester.HarvesterExtensionOut;
import frc.robot.commands.harvester.HarvesterIntake;

/**
 * Deploys the harvester and runs the harvester motors.
 */
public class HarvesterClawIntake extends SequentialCommandGroup {
    /**
     * Creates a new HarvestRecordIntake command.
     * 
     * @param isCone If the harvester is grabbing a cone.
     */
    public HarvesterClawIntake (boolean isCone) {
        addCommands(
            // Deploy the harvester.
            new HarvesterExtensionOut(),

            // Run the intake motors.
            new HarvesterIntake(isCone),

            // If picking up a cone, close the claw to utilize the flanges to knock over upright cones in the record player.
            new ConditionalCommand(
                new ClawClose(true),
                new InstantCommand(),
                () -> isCone
            ),

            // Move the arm back.
            new ArmToPosition(ArmConstants.INTERNAL_DEFAULT).withTimeout(3)
        );
    }
}
