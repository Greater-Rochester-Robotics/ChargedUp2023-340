// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Timer;
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

    double elbow1Radius = 0;
    double elbow2Radius = 0;

    double elbow1Mass = 0;
    double elbow2Mass = 0;

    double elbow1Inertia = elbow1Mass * elbow1Radius;
    double elbow2Inertia = elbow2Mass * elbow2Radius;    
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

    }

    @Override
    public void periodic () {
        // Get the current positions of the shoulder and elbow.
        double elbowPos = getElbowPosition();

        // If the elbow is no longer in a safe position, stop it.
        // if (Math.abs(elbowPos) > ArmConstants.MAX_ELBOW_ANGLE) {
        //     System.out.println("Elbow is no longer at a safe angle, halting movement");
        //     stopElbow();
        // }

        // Publish values to network tables.
        if (RobotContainer.shouldPublishToNet()) {
            netTable.getEntry("elbow").setDouble(Math.round(elbowPos * 10) / 10);
            netTable.getEntry("wrist").setBoolean(this.isWristExtended());

            SmartDashboard.putNumber("Absolute encoder elbow", Math.round(Math.toDegrees(elbowPos) * 10) * 0.1);
        }
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
        elbowPID.setReference(targetAngle + Math.PI, ControlType.kPosition, 0, gravityCounterConstant * Math.sin(getElbowPosition() - ArmConstants.SHOULDER_FIXED_ANGLE));
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
            getElbowPosition(),
            wrist.get().equals(Value.kForward)
        );
    }
}
