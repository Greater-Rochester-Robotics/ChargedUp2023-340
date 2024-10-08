// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawIntakeSlow;
import frc.robot.commands.claw.ClawStop;

/**
 * Finishes picking up a game piece from the record player.
 * If a cone, stop the claw intake, close the claw, and retract the wrist.
 * If a cube, run the claw intake slowly and retract the wrist.
 */
public class ClawWristRetract extends SequentialCommandGroup {
    /**
     * Creates a new ClawWristRetract command.
     * 
     * @param isCone If the claw is grabbing a cone.
     */
    public ClawWristRetract (boolean isCone) {
        addCommands(
            // If a cone, stop the claw intake, close the claw, and wait to ensure the claw has fully grabbed the cone.
            // If a cube, switch to running the claw intake slowly.
            new ClawStop(),
            new ConditionalCommand(new ClawClose(true), new ClawIntakeSlow(), () -> isCone),//TODO: Change once SHUTTLE VALVE INSTALLED
            new WaitCommand(0.3),
            
            new ConditionalCommand(new ArmToPosition(ArmConstants.INTERNAL_CONE), new ArmToPosition(ArmConstants.INTERNAL_CUBE), () -> isCone)
        );
    }
}
