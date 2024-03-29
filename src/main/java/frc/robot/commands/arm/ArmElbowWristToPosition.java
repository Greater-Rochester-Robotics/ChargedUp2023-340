// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmPosition;
import frc.robot.RobotContainer;

/**
 * Moves the wrist to a position.
 */
public class ArmElbowWristToPosition extends SequentialCommandGroup {
    /**
     * Creates a new ArmWristManual command.
     * @param target The target position to move to.
     */
    public ArmElbowWristToPosition (ArmPosition target) {
        
        double elbowTarget = target.getElbowPosition();
        addCommands(
            // Home the wrist
            new ArmWristHome(),

            // Check if arm is in front of harvester going back
            new ConditionalCommand(
                Commands.sequence(
                    new PrintCommand("is in front of harvester going back"),
                    // Move arm in front of harvester
                    new ArmElbowWristToPositionPID(new ArmPosition(ArmConstants.ELBOW_ANGLE_IN_FRONT_OF_HARVESTER, 0)),
                    new ConditionalCommand(
                        // Move arm behind robot
                        new ParallelCommandGroup(
                            new ArmElbowWristToPositionPID(new ArmPosition(ArmConstants.ELBOW_ANGLE_BEHIND_ROBOT, 0)),
                            new PrintCommand("is going behind robot, moving to ELBOW_ANGLE_BEHIND_ROBOT")
                        ),
                        // Move arm behind harvester
                        new ParallelCommandGroup(
                            new ArmElbowWristToPositionPID(new ArmPosition(ArmConstants.ELBOW_ANGLE_BEHIND_HARVESTER, 0)),
                            new PrintCommand("is not going behind robot, moving to ELBOW_ANGLE_BEHIND_HARVESTER")
                        ),
                        () -> isGoingBehindRobot(elbowTarget)
                    )
                ),
                // Check if arm is in the robot going out
                new ConditionalCommand(
                    Commands.sequence(
                        new PrintCommand("isn't in front of harvester going back\nis in robot going out"),
                        // Retract wrist (don't know direction to set up)
                        new ArmWristToPosition(0),
                        new PrintCommand("has retracted wrist"),
                        new ConditionalCommand(
                            // Move arm in front of harvester
                            new ParallelCommandGroup(
                                new ArmElbowWristToPositionPID(new ArmPosition(ArmConstants.ELBOW_ANGLE_IN_FRONT_OF_HARVESTER, 0)),
                                new PrintCommand("Is going in front of harvester, moving to ELBOW_ANGLE_IN_FRONT_OF_HARVESTER")
                            ),
                            // Move arm behind robot
                            new ParallelCommandGroup(
                                new ArmElbowWristToPositionPID(new ArmPosition(ArmConstants.ELBOW_ANGLE_BEHIND_ROBOT, 0)),
                                new PrintCommand("Is going behind harvester, moving to ELBOW_ANGLE_BEHIND_ROBOT")
                            ),
                            () -> isGoingInFrontOfHarvester(elbowTarget)
                        )
                    ),
                    // Check if arm is behind robot going forward
                    new ConditionalCommand(
                        Commands.sequence(
                            new PrintCommand("isn't in front of harvester going back\nisn't in robot going out"),
                            // Move arm to back of robot
                            new PrintCommand("moving to ELBOW_ANGLE_BEHIND_ROBOT, and retracting wrist"),
                            new ArmElbowWristToPositionPID(new ArmPosition(ArmConstants.ELBOW_ANGLE_BEHIND_ROBOT, 0)),
                            new ConditionalCommand(
                                // Move arm in front of harvester
                                new ArmElbowWristToPositionPID(new ArmPosition(ArmConstants.ELBOW_ANGLE_IN_FRONT_OF_HARVESTER, 0)),
                                // Move arm behind record player (closest safe zone)
                                new ArmElbowWristToPositionPID(new ArmPosition(ArmConstants.ELBOW_ANGLE_BEHIND_RECORD_PLAYER, 0)),
                                () -> isGoingInFrontOfHarvester(elbowTarget)
                            )
                        ),
                        // Do nothing (not going to do unsafe motion)
                        new PrintCommand("no unsafe motion"),
                        () -> isBehindRobotGoingForward(RobotContainer.arm.getElbowPosition(), elbowTarget)
                    ),
                    () -> isInRobotGoingOut(RobotContainer.arm.getElbowPosition(), elbowTarget)
                ),
                () -> isInFrontOfHarvesterGoingBack(RobotContainer.arm.getElbowPosition(), elbowTarget)
            ),

            // Move to final arm position
            new ArmElbowWristToPositionPID(target)
        );
    }

    /**
     * Moves the wrist to position using PID.
     */
    private class ArmElbowWristToPositionPID extends CommandBase {
        /**
         * If the elbow is going down.
         */
        private boolean goingDown;

        /**
         * A counter for the number of times the target position has been hit to combat slop.
         * If this value exceeds 10, the wrist is assumed to be at position.
         * Incremented if the arm is within tolerance when execute() is called.
         */
        private int wristHitTarget;
        private int elbowHitTarget;

        /**
         * The target position to move to.
         */
        private ArmPosition target;

        /**
         * Creates a new ArmWristToPositionPID command.
         * @param target The target position to move to.
         */
        public ArmElbowWristToPositionPID (ArmPosition target) {
            addRequirements(RobotContainer.arm);
            this.target = target;
        }

        @Override
        public void initialize () {
            // Set helpers.
            wristHitTarget = 0;
            elbowHitTarget = 0;
            goingDown = Math.abs(RobotContainer.arm.getElbowPosition()) > Math.abs(target.getElbowPosition());
        }

        @Override
        public void execute () {
            // Set the target
            RobotContainer.arm.setWristPosition(target.getWristPosition());
            RobotContainer.arm.setElbowPosition(target.getElbowPosition());

            // If the wrist is within tolerance of the target position, increment hitTarget. Otherwise, reset the hitTarget count.
            if (Math.abs(RobotContainer.arm.getWristPosition() - target.getWristPosition()) < ArmConstants.WRIST_CLOSED_LOOP_ERROR) {
                wristHitTarget++;
            } else {
                wristHitTarget = 0;
            }

            if (Math.abs(RobotContainer.arm.getElbowPosition() - target.getElbowPosition()) < ArmConstants.ELBOW_CLOSED_LOOP_ERROR) {
                elbowHitTarget++;
            } else {
                elbowHitTarget = 0;
            }
        }

        @Override
        public void end (boolean interrupted) {
        // If ended, stop the wrist from moving.
            RobotContainer.arm.stopWristMotor();
            if(RobotContainer.arm.getWristInnerLimitSwitch() && !interrupted){
                RobotContainer.arm.zeroWrist();
            }
        }

        @Override
        public boolean isFinished () {
            return wristIsFinished() && elbowIsFinished();
        }

        private boolean wristIsFinished() {
            // Finish if hitTarget has been incremented to 10, or if the wrist has exceeded the maximum safe extension.
            return wristHitTarget >= ArmConstants.WRIST_TARGET_HIT_COUNT || RobotContainer.arm.getWristOuterLimitSwitch() || 
                (target.getWristPosition() < .025 &&  RobotContainer.arm.getWristInnerLimitSwitch());    
        }
        private boolean elbowIsFinished() {
            return elbowHitTarget >= ArmConstants.ELBOW_TARGET_HIT_COUNT || (!goingDown && Math.abs(RobotContainer.arm.getElbowPosition()) > ArmConstants.MAX_ELBOW_ANGLE);
        }
    }

    /* Current Pos Checks */
    private boolean isInFrontOfHarvesterGoingBack(double elbowPos, double target) {
        return elbowPos > ArmConstants.ELBOW_ANGLE_IN_FRONT_OF_HARVESTER && target < ArmConstants.ELBOW_ANGLE_BEHIND_HARVESTER;
    }

    private boolean isInRobotGoingOut(double elbowPos, double target) {
        return (elbowPos < ArmConstants.ELBOW_ANGLE_BEHIND_HARVESTER && elbowPos > ArmConstants.ELBOW_ANGLE_BEHIND_RECORD_PLAYER) && (target > ArmConstants.ELBOW_ANGLE_IN_FRONT_OF_HARVESTER || target < ArmConstants.ELBOW_ANGLE_BEHIND_ROBOT);
    }

    private boolean isBehindRobotGoingForward(double elbowPos, double target) {
        return elbowPos < ArmConstants.ELBOW_ANGLE_BEHIND_ROBOT && target > ArmConstants.ELBOW_ANGLE_BEHIND_RECORD_PLAYER;
    }

    /* Target Pos Checks */
    private boolean isGoingInFrontOfHarvester(double target) {
        return target > ArmConstants.ELBOW_ANGLE_IN_FRONT_OF_HARVESTER;
    }

    private boolean isGoingBehindRobot(double target) {
        return target < ArmConstants.ELBOW_ANGLE_BEHIND_ROBOT;
    }
}
