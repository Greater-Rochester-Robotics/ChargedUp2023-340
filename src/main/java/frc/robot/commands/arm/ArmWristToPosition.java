// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;

/**
 * Moves the wrist to a position.
 */
public class ArmWristToPosition extends SequentialCommandGroup {
    /**
     * Creates a new ArmWristManual command.
     * @param target The target position to move to.
     */
    public ArmWristToPosition (double target) {
        addCommands(
            new ArmWristHome(),
            new ArmWristToPositionPID(target)
        );
    }

    /**
     * Moves the wrist to position using PID.
     */
    private class ArmWristToPositionPID extends CommandBase {
        /**
         * A counter for the number of times the target position has been hit to combat slop.
         * If this value exceeds 10, the wrist is assumed to be at position.
         * Incremented if the arm is within tolerance when execute() is called.
         */
        private int hitTarget;
        /**
         * The target position to move to.
         */
        private double target;

        /**
         * Creates a new ArmWristToPositionPID command.
         * @param target The target position to move to.
         */
        public ArmWristToPositionPID (double target) {
            addRequirements(RobotContainer.arm);
            this.target = target;
        }

        @Override
        public void initialize () {
            // Set helpers.
            hitTarget = 0;
            RobotContainer.arm.resetWristController();
        }

        @Override
        public void execute () {
            // Set the target,
            RobotContainer.arm.setWristPosition(target);

            // If the wrist is within tolerance of the target position, increment hitTarget. Otherwise, reset the hitTarget count.
            if (Math.abs(RobotContainer.arm.getWristPosition() - target) < ArmConstants.WRIST_CLOSED_LOOP_ERROR) {
                hitTarget++;
            } else {
                hitTarget = 0;
            }
        }

        @Override
        public void end (boolean interrupted) {
        // If ended, stop the wrist from moving.
            RobotContainer.arm.stopWristMotor();
            if(RobotContainer.arm.getWristInnerLimitSwitch()){
                RobotContainer.arm.zeroWrist();
            }
        }

        @Override
        public boolean isFinished () {
            // Finish if hitTarget has been incremented to 10, or if the wrist has exceeded the maximum safe extension.
            return hitTarget >= 5 || RobotContainer.arm.getWristOuterLimitSwitch() || 
                (target < .025 && RobotContainer.arm.getWristInnerLimitSwitch());         
            
        }
    }
}
