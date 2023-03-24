// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.awt.geom.AffineTransform;
import java.awt.geom.Point2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ArmPosition {
    // use Rotation2ds instead of double angles?
    // find where angles are measured from (relative to the ground or relative to something else?)
    private double shoulderAngle;
    private double elbowAngle;
    private double wristLength;

    public ArmPosition (){
        shoulderAngle = 0;
        elbowAngle = 0;
        wristLength = 0;
    }

    public ArmPosition (double elbowAngle, double wristLength) {
        this.shoulderAngle = ArmConstants.SHOULDER_FIXED_ANGLE;
        this.elbowAngle = elbowAngle;
        this.wristLength = wristLength;
    }

    public double getElbowPosition () {
        return elbowAngle;
    }

    public double getWristPosition () {
        return wristLength;
    }

    public ArmPosition getAsArmPosition () {
        return this;
    }

    public Pose2d getEndPosition () {
        double elbowToEnd = ArmConstants.ELBOW_TO_CLAW_DISTANCE + wristLength;
        return(new Pose2d(
            Math.cos(elbowAngle)*elbowToEnd,
            Math.sin(elbowAngle)*elbowToEnd,
            new Rotation2d()
        ));
    }

    /**
     * measured from the shoulder pivot
     * 
     * @return
     */
    public double getEndX () {
        double elbowToEnd = ArmConstants.ELBOW_TO_CLAW_DISTANCE + wristLength;
        return (Math.cos(elbowAngle)*elbowToEnd);
    }

    public boolean isInFrontOfHarvester () {
        return getEndX() > ArmConstants.ARM_TO_HARVESTER_MAX_DISTANCE;
    }

    public boolean isBehindHarvester () {
        return getEndX() < ArmConstants.ARM_TO_HARVESTER_MIN_DISTANCE;
    }

    public boolean isOppositeSideFromCurrent () {
        ArmPosition currentPosition = RobotContainer.arm.getArmPosition();
        return !((this.isBehindHarvester()
            && currentPosition.isBehindHarvester())
            || (this.isInFrontOfHarvester()
                && currentPosition.isInFrontOfHarvester()));
    }

    public static ArmPosition inverseKinematics (double x, double z) {
        double shoulderToElbow = ArmConstants.SHOULDER_TO_ELBOW_DISTANCE;
        double elbowToEnd = ArmConstants.ELBOW_TO_CLAW_DISTANCE;
        double shoulderToEnd = Math.sqrt(x
            * x
            + z
                * z); // Gets the distance between the shoulder joint of the arm and the end point
        boolean isExtended = false;
        if (shoulderToEnd > shoulderToElbow
            + elbowToEnd) {
            elbowToEnd += ArmConstants.WRIST_MAX_EXTENSION_LENGTH;
            isExtended = true;
        }
        return new ArmPosition();
    }
    public ArmPosition interpolateArmPosition (ArmPosition nextPosition, double time) {
        return interpolateArmPosition(this, nextPosition, time);
    }

    public static ArmPosition interpolateArmPosition (ArmPosition firstPosition, ArmPosition secondPosition, double time) {
        return new ArmPosition(
            (firstPosition.elbowAngle*time)+(secondPosition.elbowAngle*(1-time)), 
            (firstPosition.wristLength*time)+(secondPosition.wristLength*(1-time))
            );
    }
}
