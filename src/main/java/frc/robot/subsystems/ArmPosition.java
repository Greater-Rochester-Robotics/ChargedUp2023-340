// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/**
 * An arm position.
 */
public class ArmPosition {
    /**
     * The elbow's angle in radians.
     */
    private double elbowAngle;
    /**
     * The wrist's length.
     */
    private double wristLength;

    /**
     * Create a new ArmPosition.
     * Has an elbow angle and wrist length of 0.
     */
    public ArmPosition () {
        this(0, 0);
    }

    /**
     * Create a new ArmPosition.
     * @param elbowAngle The elbow's angle in radians.
     * @param wristLength The wrist's length.
     */
    public ArmPosition (double elbowAngle, double wristLength) {
        this.elbowAngle = elbowAngle;
        this.wristLength = wristLength;
    }

    /**
     * Gets the elbow's angle.
     * @return The elbow's angle in radians.
     */
    public double getElbowAngle () {
        return elbowAngle;
    }

    /**
     * Gets the wrist's length.
     * @return The wrist's length.
     */
    public double getWristLength () {
        return wristLength;
    }
}
