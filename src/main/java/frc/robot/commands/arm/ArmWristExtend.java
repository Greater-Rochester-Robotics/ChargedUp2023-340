// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;

/**
 * Extends the wrist.
 */
public class ArmWristExtend extends CommandBase {
    /**
     * The timer used to wait for the wrist to be extended, as the robot only
     * knows the robot's solenoid states, not the affected piston's current position.
     */
    private Timer timer = new Timer();
    /**
     * If the command should wait for the wrist to be extended to finish.
     */
    private boolean waitForExtend;

    /**
     * Creates a new ArmWristExtend command.
     * 
     * @param waitForExtend If the command should wait for the wrist to be extended to finish.
     * @see ArmConstants#WRIST_EXTENSION_DELAY
     */
    public ArmWristExtend (boolean waitForExtend) {
        addRequirements(RobotContainer.arm);
        this.waitForExtend = waitForExtend;
    }

    @Override
    public void initialize () {
        // Start the timer.
        timer.reset();
        timer.start();

        // Extend the wrist.
        RobotContainer.arm.extendWrist();
    }

    @Override
    public boolean isFinished () {
        // Finish if the wrist is extended, or if the command ignores if the wrist should be extended.
        return timer.hasElapsed(ArmConstants.WRIST_EXTENSION_DELAY) || !waitForExtend;
    }
}
