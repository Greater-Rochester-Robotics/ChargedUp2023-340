// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;

/**
 * Retracts the wrist.
 */
public class ArmWristRetract extends InstantCommand {
    /**
     * The timer used to wait for the wrist to be retracted, as the robot only
     * knows the robot's solenoid states, not the affected piston's current position.
     */
    Timer timer = new Timer();
    /**
     * If the command should wait for the wrist to be retracted to finish.
     */
    boolean waitForRetract;

    /**
     * Creates a new ArmWristRetract command.
     * 
     * @param waitForRetract If the command should wait for the wrist to be retracted to finish.
     * @see ArmConstants#WRIST_RETRACTION_DELAY
     */
    public ArmWristRetract (boolean waitForRetract) {
        addRequirements(RobotContainer.arm);
        this.waitForRetract = waitForRetract;
    }

    @Override
    public void initialize () {
        // Start the timer.
        timer.reset();
        timer.start();

        // Retract the wrist.
        RobotContainer.arm.retractWrist();
    }

    @Override
    public boolean isFinished () {
        // Finish if the wrist is retracted, or if the command ignores if the wrist should be retracted.
        return timer.hasElapsed(ArmConstants.WRIST_RETRACTION_DELAY) || !waitForRetract;
    }
}
