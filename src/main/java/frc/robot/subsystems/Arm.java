// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

/**
 * The arm subsystem.
 * Controls the shoulder, elbow, and wrist.
 */
public class Arm extends SubsystemBase {
    /**
     * The right shoulder motor.
     */
    private CANSparkMax shoulderRight;
    /**
     * The right shoulder's absolute encoder.
     */
    private AbsoluteEncoder shoulderRightEncoder;
    /**
     * The right shoulder's PID controller.
     */
    private SparkMaxPIDController shoulderRightPID;
    /**
     * The right shoulder's profiled PID controller.
     */
    private ProfiledPIDController shoulderRightProPID;

    /**
     * The left shoulder motor.
     */
    private CANSparkMax shoulderLeft;
    /**
     * The left shoulder's absolute encoder.
     */
    private AbsoluteEncoder shoulderLeftEncoder;
    /**
     * The left shoulder's PID controller.
     */
    private SparkMaxPIDController shoulderLeftPID;
    /**
     * The left shoulder's profiled PID controller.
     */
    private ProfiledPIDController shoulderLeftProPID;

    /**
     * The elbow motor.
     */
    private CANSparkMax elbow;
    /**
     * The elbow's absolute encoder.
     */
    private AbsoluteEncoder elbowEncoder;
    /**
     * The elbow's PID controller.
     */
    private SparkMaxPIDController elbowPID;
    /**
     * The elbow break.
     */
    private Solenoid elbowBrake;

    /**
     * The wrist solenoid.
     */
    private DoubleSolenoid wrist;

    /**
     * The shoulder's target position.
     */
    private double shoulderTarget;
    /**
     * If the shoulder should move to its target.
     */
    private boolean moveToShoulderTarget;

    /**
     * The network table instance used by the arm subsystem.
     */
    private NetworkTableInstance netInstance = NetworkTableInstance.getDefault();
    /**
     * The network table used by the arm subsystem.
     */
    private NetworkTable netTable = netInstance.getTable("/dashboard/robotmodel");

    /**
     * Creates a new Arm subsystem.
     */
    public Arm () {
        // Setup the right shoulder.
        shoulderRight = new CANSparkMax(Constants.SHOULDER_MOTOR_RIGHT, MotorType.kBrushless);
        shoulderRightEncoder = shoulderRight.getAbsoluteEncoder(Type.kDutyCycle);
        shoulderRightPID = shoulderRight.getPIDController();
        shoulderRightProPID = new ProfiledPIDController(
            ArmConstants.SHOULDER_P_RIGHT,
            ArmConstants.SHOULDER_I_RIGHT,
            ArmConstants.SHOULDER_D_RIGHT,
            new Constraints(ArmConstants.MAX_SHOULDER_VELOCITY, ArmConstants.MAX_SHOULDER_ACCELERATION)
        );

        // Right shoulder motor settings.
        shoulderRight.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
        shoulderRight.setInverted(false);
        shoulderRight.setIdleMode(IdleMode.kBrake);
        shoulderRight.setClosedLoopRampRate(0.25);

        // Right shoulder frame settings.
        shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
        shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
        shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 59509);
        shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 59497);
        shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 7);
        shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10);

        // Right shoulder encoder settings.
        shoulderRightEncoder.setPositionConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONV_FACTOR);
        shoulderRightEncoder.setVelocityConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONV_FACTOR / 60);
        shoulderRightEncoder.setInverted(false);
        shoulderRightEncoder.setZeroOffset(1.7072);

        // Right shoulder PID settings.
        shoulderRightPID.setFeedbackDevice(shoulderRightEncoder);
        shoulderRightPID.setOutputRange(-ArmConstants.MAX_SHOULDER_OUT_PID_OUT, ArmConstants.MAX_SHOULDER_IN_PID_OUT);
        shoulderRightPID.setP(ArmConstants.SHOULDER_P_RIGHT);
        shoulderRightPID.setI(ArmConstants.SHOULDER_I_RIGHT);
        shoulderRightPID.setD(ArmConstants.SHOULDER_D_RIGHT);
        shoulderRightPID.setFF(ArmConstants.SHOULDER_F_RIGHT);


        // Setup the left shoulder.
        shoulderLeft = new CANSparkMax(Constants.SHOULDER_MOTOR_LEFT, MotorType.kBrushless);
        shoulderLeftEncoder = shoulderLeft.getAbsoluteEncoder(Type.kDutyCycle);
        shoulderLeftPID = shoulderLeft.getPIDController();
        shoulderLeftProPID = new ProfiledPIDController(
            ArmConstants.SHOULDER_P_LEFT,
            ArmConstants.SHOULDER_I_LEFT,
            ArmConstants.SHOULDER_D_LEFT,
            new Constraints(ArmConstants.MAX_SHOULDER_VELOCITY, ArmConstants.MAX_SHOULDER_ACCELERATION)
        );

        // Left shoulder motor settings.
        shoulderLeft.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
        shoulderLeft.setInverted(false);
        shoulderLeft.setIdleMode(IdleMode.kBrake);
        shoulderLeft.setClosedLoopRampRate(0.25);

        // Left shoulder frame settings.
        shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
        shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
        shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 59473);
        shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 59471);
        shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 7);
        shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10);

        // Left shoulder encoder settings.
        shoulderLeftEncoder.setPositionConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONV_FACTOR);
        shoulderLeftEncoder.setVelocityConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONV_FACTOR / 60);
        shoulderLeftEncoder.setInverted(true);
        shoulderLeftEncoder.setZeroOffset(3.3257);

        // Left shoulder PID settings.
        shoulderLeftPID.setFeedbackDevice(shoulderLeftEncoder);
        shoulderLeftPID.setOutputRange(-ArmConstants.MAX_SHOULDER_OUT_PID_OUT, ArmConstants.MAX_SHOULDER_IN_PID_OUT);
        shoulderLeftPID.setP(ArmConstants.SHOULDER_P_LEFT);
        shoulderLeftPID.setI(ArmConstants.SHOULDER_I_LEFT);
        shoulderLeftPID.setD(ArmConstants.SHOULDER_D_LEFT);
        shoulderLeftPID.setFF(ArmConstants.SHOULDER_F_LEFT);


        // Setup the elbow.
        elbow = new CANSparkMax(Constants.ELBOW_MOTOR, MotorType.kBrushless);
        elbowEncoder = elbow.getAbsoluteEncoder(Type.kDutyCycle);
        elbowPID = elbow.getPIDController();
        elbowBrake = new Solenoid(PneumaticsModuleType.REVPH, Constants.ELBOW_BRAKE);

        // Elbow motor settings.
        elbow.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
        elbow.setInverted(true);
        elbow.setIdleMode(IdleMode.kBrake);
        elbow.setClosedLoopRampRate(1);

        // Elbow frame settings.
        elbow.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
        elbow.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        elbow.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        elbow.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 59467);
        elbow.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 59453);
        elbow.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 6);
        elbow.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10);

        // Elbow encoder settings.
        elbowEncoder.setPositionConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONV_FACTOR);
        elbowEncoder.setVelocityConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONV_FACTOR / 60);
        elbowEncoder.setInverted(true);
        elbowEncoder.setZeroOffset(5.8760048535897932384626433832795);

        // Elbow PID settings.
        elbowPID.setFeedbackDevice(elbowEncoder);
        elbowPID.setOutputRange(-ArmConstants.MAX_ELBOW_PID_OUT, ArmConstants.MAX_ELBOW_PID_OUT);
        elbowPID.setPositionPIDWrappingEnabled(false);
        elbowPID.setP(ArmConstants.ELBOW_P);
        elbowPID.setI(ArmConstants.ELBOW_I);
        elbowPID.setD(ArmConstants.ELBOW_D);
        elbowPID.setFF(ArmConstants.ELBOW_F);

        // Setup the wrist.
        wrist = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.WRIST_SOLENOID_OUT, Constants.WRIST_SOLENOID_IN);
        retractWrist();

        // Start the elbow at 0 speed.
        elbow.set(0);

        // Burn flash.
        shoulderRight.burnFlash();
        shoulderLeft.burnFlash();
        elbow.burnFlash();

        // Set the shoulder target to the current position..
        shoulderTarget = getShoulderPosition();

        // The shoulder should not move to target on startup.
        moveToShoulderTarget = false;
    }

    @Override
    public void periodic () {
        // Get the current positions of the shoulder and elbow.
        double rightPos = getRightShoulderPosition();
        double leftPos = getLeftShoulderPosition();
        double elbowPos = getElbowPosition();

        // If the shoulder is no longer in a safe position, stop it.
        // if (rightPos > ArmConstants.MAX_SHOULDER_ANGLE || leftPos > ArmConstants.MAX_SHOULDER_ANGLE || rightPos < ArmConstants.MIN_SHOULDER_ANGLE || leftPos < ArmConstants.MIN_SHOULDER_ANGLE) {
        //     System.out.println("Shoulder is no longer at a safe angle, halting movement");
        //     stopShoulder();
        // }

        // If the elbow is no longer in a safe position, stop it.
        // if (Math.abs(elbowPos) > ArmConstants.MAX_ELBOW_ANGLE) {
        //     System.out.println("Elbow is no longer at a safe angle, halting movement");
        //     stopElbow();
        // }

         // If the shoulder should move to the target, continue to move it.
        if (moveToShoulderTarget) {
            // If the shoulders are out of sync.
            if (rightPos - leftPos > ArmConstants.SHOULDER_ALLOWED_DIFFERENCE) {
                double rightDiff = shoulderTarget - rightPos;
                double leftDiff = shoulderTarget - leftPos;

                // Move a single shoulder.
                if (Math.abs(rightDiff) > Math.abs(leftDiff)) {
                    shoulderLeft.set(0.0);
                    setRightShoulderPosition(shoulderTarget);
                } else {
                    shoulderRight.set(0.0);
                    setLeftShoulderPosition(shoulderTarget);
                }
            } else {
                // Move the shoulders towards the target in sync.
                setLeftShoulderPosition(shoulderTarget);
                setRightShoulderPosition(shoulderTarget);
            }
        }

        // Publish values to network tables.
        if (RobotContainer.shouldPublishToNet()) {
            netTable.getEntry("shoulder").setDouble(Math.round(getShoulderPosition() * 10) / 10);
            netTable.getEntry("elbow").setDouble(Math.round(elbowPos * 10) / 10);
            netTable.getEntry("wrist").setBoolean(this.isWristExtended());

            SmartDashboard.putNumber("Absolute encoder right", Math.round(Math.toDegrees(rightPos) * 10) * 0.1);
            SmartDashboard.putNumber("Absolute encoder left", Math.round(Math.toDegrees(leftPos) * 10) * 0.1);
            SmartDashboard.putNumber("Absolute encoder elbow", Math.round(Math.toDegrees(elbowPos) * 10) * 0.1);
        }
    }

    /**
     * Gets the left shoulder's position in radians.
     * @return The left shoulder's position in radians.
     */
    public double getLeftShoulderPosition () {
        return shoulderLeftEncoder.getPosition() - Math.PI;
    }

    /**
     * Gets the right shoulder's position in radians.
     * @return The right shoulder's position in radians.
     */
    public double getRightShoulderPosition () {
        return shoulderRightEncoder.getPosition()
            - Math.PI;
    }

    /**
     * Gets the shoulder's position in radians by taking the average of the left and right shoulder positions.
     * @return The shoulder's position in radians.
     */
    public double getShoulderPosition () {
        return (getRightShoulderPosition() + getLeftShoulderPosition()) / 2;
    }

    /**
     * Gets the velocity of the left shoulder in radians / second.
     * @return The left shoulder's velocity in radians / second.
     * @unused
     */
    public double getLeftShoulderVelocity () {
        return shoulderLeftEncoder.getVelocity();
    }

    /**
     * Gets the velocity of the right shoulder in radians / second.
     * @return The right shoulder's velocity in radians / second.
     * @unused
     */
    public double getRightShoulderVelocity () {
        return shoulderRightEncoder.getVelocity();
    }

    /**
     * Gets the velocity of the shoulder in radians / second by taking the average of the left and right shoulder velocities.
     * @return The shoulder's velocity in radians / second.
     * @unused
     */
    public double getShoulderVelocity () {
        return (getLeftShoulderVelocity() + getRightShoulderVelocity()) / 2;
    }

    /**
     * Sets the right shoulder encoder's offset.
     * @param offset The offset to use.
     * @unused
     */
    public void setRightShoulderOffset (double offset) {
        shoulderRightEncoder.setZeroOffset(offset);
        shoulderRight.burnFlash();
    }

    /**
     * Sets the left shoulder encoder's offset.
     * @param offset The offset to use.
     * @unused
     */
    public void setLeftShoulderOffset (double offset) {
        shoulderLeftEncoder.setZeroOffset(offset);
        shoulderLeft.burnFlash();
    }

    /**
     * Drives the left shoulder via duty cycle.
     * @param speed The speed to set the motor to, should be a value between -1.0 and 1.0.
     */
    public void setLeftShoulderDutyCycle (double speed) {
        shoulderLeft.set(speed);
        moveToShoulderTarget = false;
    }

    /**
     * Drives the right shoulder via duty cycle.
     * @param speed The speed to set the motor to, should be a value between -1.0 and 1.0.
     */
    public void setRightShoulderDutyCycle (double speed) {
        shoulderRight.set(speed);
        moveToShoulderTarget = false;
    }

    /**
     * Sets the left shoulder's position.
     * @param targetAngle The target angle in radians.
     */
    public void setLeftShoulderPosition (double targetAngle) {
        // Check if the specified angle is out of range.
        if (targetAngle > ArmConstants.MAX_SHOULDER_ANGLE || targetAngle < ArmConstants.MIN_SHOULDER_ANGLE) {
            System.out.println("Cannot set left shoulder target to " + Math.round(Math.toDegrees(targetAngle)) + " degrees: Out of Range");
            return;
        }

        // Set the target angle in the PID controller.
        shoulderLeft.getPIDController().setReference(targetAngle + Math.PI, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Sets the right shoulder's position.
     * @param targetAngle The target angle in radians.
     */
    public void setRightShoulderPosition (double targetAngle) {
        // Check if the specified angle is out of range.
        if (targetAngle > ArmConstants.MAX_SHOULDER_ANGLE || targetAngle < ArmConstants.MIN_SHOULDER_ANGLE) {
            System.out.println("Cannot set right shoulder target to " + Math.round(Math.toDegrees(targetAngle)) + " degrees: Out of Range");
            return;
        }

        // Set the target angle in the PID controller.
        shoulderRight.getPIDController().setReference(targetAngle + Math.PI, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Sets the shoulder's position.
     * @param targetAngle The target angle in radians.
     */
    public void setShoulderPosition (double targetAngle) {
        // Set the positions on both shoulders.
        setLeftShoulderPosition(targetAngle);
        setRightShoulderPosition(targetAngle);

        // Set the shoulder's target to the specified angle.
        shoulderTarget = targetAngle;

        // Enable moving the shoulder in the subsystem's periodic method.
        moveToShoulderTarget = true;
    }

    /**
     * Drives both shoulder motors using profiled
     * PID controllers, uses feed forward based on
     * difference in shoulder angles to sync shoulders
     * 
     * @param targetAngle The target angle in radians.
     * @unused
     */
    public void setShoulderPositionPID (double targetAngle) {
        double leftSensor = getLeftShoulderPosition();
        double rightSensor = getRightShoulderPosition();
        double diff = leftSensor - rightSensor;

        shoulderLeft.set(shoulderLeftProPID.calculate(getLeftShoulderPosition(), targetAngle) + ArmConstants.SHOULDER_F_DIFFERENCE * diff);
        shoulderRight.set(shoulderRightProPID.calculate(getRightShoulderPosition(), targetAngle) + ArmConstants.SHOULDER_F_DIFFERENCE * diff);
    }

    /**
     * Stops both shoulder motors.
     */
    public void stopShoulder () {
        shoulderRight.set(0);
        shoulderLeft.set(0);
        moveToShoulderTarget = false;
    }

    /**
     * Gets the elbow's position in radians.
     * @return The elbow's position in radians.
     */
    public double getElbowPosition () {
        return elbowEncoder.getPosition() - Math.PI;
    }

    /**
     * Gets the velocity of the elbow in radians / second.
     * @return The elbow's velocity in radians / second.
     * @unused
     */
    public double getElbowVelocity () {
        return elbowEncoder.getVelocity();
    }

    /**
     * Drives the elbow via duty cycle.
     * @param speed The speed to set the motor to, should be a value between -1.0 and 1.0.
     */
    public void setElbowDutyCycle (double speed) {
        elbow.set(speed);
        elbowBrake.set(true);
    }

    /**
     * Sets the elbow's position.
     * @param targetAngle The target angle in radians.
     */
    public void setElbowPosition (double targetAngle) {
        // Check if the specified angle is out of range.
        if (Math.abs(targetAngle) > ArmConstants.MAX_ELBOW_ANGLE) {
            System.out.println("Cannot set elbow target to " + Math.round(Math.toDegrees(targetAngle)) + " degrees: Out of Range");
            return;
        }

        // Counter for gravity.
        double gravityCounterConstant = isWristExtended() ? ArmConstants.KG_WRIST_OUT : ArmConstants.KG_WRIST_IN;

        // Set the target angle in the PID controller.
        elbowPID.setReference(targetAngle + Math.PI, ControlType.kPosition, 0, gravityCounterConstant * Math.sin(getElbowPosition() - getShoulderPosition()));
        elbowBrake.set(true);
    }

    /**
     * Stops the elbow and re-engages the brake.
     */
    public void stopElbow () {
        elbow.set(0);
        elbowBrake.set(false);
    }

    /**
     * Checks if the wrist is extended.
     * @return true if the wrist is extended, false otherwise.
     */
    public boolean isWristExtended () {
        return wrist.get() == Value.kForward;
    }

    /**
     * Extends the wrist.
     */
    public void extendWrist () {
        wrist.set(Value.kForward);
    }

    /**
     * Retracts the wrist.
     */
    public void retractWrist () {
        wrist.set(Value.kReverse);
    }

    /**
     * Gets the arm's current position.
     * @return
     */
    public ArmPosition getArmPosition () {
        return new ArmPosition(
            getShoulderPosition(),
            getElbowPosition(),
            wrist.get().equals(Value.kForward)
        );
    }
}
