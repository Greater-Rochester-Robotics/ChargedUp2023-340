// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;

/**
 * The arm subsystem.
 * Controls the shoulder, elbow, and wrist.
 */
public class Arm extends SubsystemBase {
    /**
     * The elbow motor.
     */
    private CANSparkMax elbow;
    /**
     * The elbow's absolute encoder.
     */
    private AbsoluteEncoder elbowEncoder;
    /**
     * The elbow's profiled PID controller.
     */
    private ProfiledPIDController elbowPID;
    /**
     * The elbow break.
     */
    private Solenoid elbowBrake;

    /**
     * The wrist motor.
     */
    private TalonSRX wrist;
    /**
     * The wrist's encoder.
     */
    private Encoder wristEncoder;
    /**
     * The wrist's PID controller.
     */
    private ProfiledPIDController wristPID;
    /**
     * The wrist's inner limit switch.
     */
    private DigitalInput wristInnerLimitSwitch;
     /**
     * The wrist's outer limit switch.
     */
    private DigitalInput wristOuterLimitSwitch;
    /**
     * If the wrist has been zeroed.
     */
    private boolean wristHasBeenZeroed = false;

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
        elbowPID = new ProfiledPIDController(ArmConstants.ELBOW_P, ArmConstants.ELBOW_I, ArmConstants.ELBOW_D, ArmConstants.ELBOW_PROFILED_PID_CONSTRAINTS);
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
        elbowEncoder.setPositionConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONVERSION_FACTOR);
        elbowEncoder.setVelocityConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONVERSION_FACTOR / 60);
        elbowEncoder.setInverted(true);
        elbowEncoder.setZeroOffset(5.8760048535897932384626433832795);

        // Setup the wrist.
        wrist = new TalonSRX(Constants.WRIST_MOTOR);
        wristPID = new ProfiledPIDController(ArmConstants.WRIST_P, ArmConstants.WRIST_I, ArmConstants.WRIST_D, ArmConstants.WRIST_PROFILED_PID_CONSTRAINTS);
        wristEncoder = new Encoder(Constants.WRIST_ENCODER_0, Constants.WRIST_ENCODER_1);
        wristInnerLimitSwitch = new DigitalInput(Constants.WRIST_INNER_LIMIT_SWITCH);
        wristOuterLimitSwitch = new DigitalInput(Constants.WRIST_OUTER_LIMIT_SWITCH);
        
        // Wrist motor settings.
        wrist.configAllSettings(new TalonSRXConfiguration());
        wrist.enableVoltageCompensation(true);
        wrist.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        wrist.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 251);
        wrist.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 254);
        wrist.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 241);
        wrist.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 237);
        wrist.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 221);
        wrist.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 211);
        
        // Wrist encoder settings.
        wristEncoder.setDistancePerPulse(ArmConstants.WRIST_ENCODER_DISTANCE_PER_PULSE);

        // Start the elbow and wrist at 0 speed.
        elbow.set(0);
        wrist.set(TalonSRXControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic () {
        // Publish values to network tables.
        if (RobotContainer.shouldPublishToNetworkTables()) {
            // Get the current positions of the shoulder and elbow.
            double elbowPos = getElbowPosition();
            double wristPos = getWristPosition();

            netTable.getEntry("elbow").setDouble(Math.round(elbowPos * 10) / 10);
            netTable.getEntry("wrist").setDouble(Math.round(wristPos * 10) / 10);

            SmartDashboard.putNumber("Absolute encoder elbow", Math.round(Math.toDegrees(elbowPos) * 10) * 0.1);
            SmartDashboard.putNumber("Wrist position", Math.round(Math.toDegrees(wristPos) * 10) * 0.1);
            SmartDashboard.putBoolean("Wrist has been zeroed", getWristBeenZeroed());
            SmartDashboard.putBoolean("Wrist limit inner", getWristInnerLimitSwitch());
            SmartDashboard.putBoolean("Wrist limit outer", getWristOuterLimitSwitch());
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

        // TODO: This will depend on wrist position (was previously two values for in and out).
        // Counter for gravity.
        double gravityCounterConstant = ArmConstants.KG;

        // Set the target angle in the PID controller.
        elbow.set(elbowPID.calculate(getElbowPosition(), targetAngle) + gravityCounterConstant * Math.sin(getElbowPosition()));

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
     * Gets the wrist's position in meters.
     * @return The wrist's position in meters.
     */
    public double getWristPosition () {
        return wristEncoder.getDistance();
    }

    /**
     * Gets the state of the wrist's inner limit switch.
     * @return The status of the inner limit switch.
     */
    public boolean getWristInnerLimitSwitch () {
        return wristInnerLimitSwitch.get();
    }

    /**
     * Gets the state of the wrist's outer limit switch.
     * @return The status of the outer limit switch.
     */
    public boolean getWristOuterLimitSwitch() {
        return wristOuterLimitSwitch.get();
    }

    /**
     * Gets if the wrist has been zeroed.
     * @return true if the wrist has been zeroed, false otherwise.
     */
    public boolean getWristBeenZeroed () {
        return wristHasBeenZeroed;
    }

    /**
     * Zeros the wrist.
     */
    public void zeroWrist () {
        wristEncoder.reset();
        wristHasBeenZeroed = true;
    }

    /**
     * Drives the wrist via duty cycle.
     * @param speed The speed to set the motor to, should be a value between -1.0 and 1.0.
     */
    public void setWristVoltage (double speed) {
        if(speed < 0 && getWristInnerLimitSwitch()) {
            System.out.println("Cannot set wrist motor speed: At lower limit");
            zeroWrist();
            speed = 0;
        }
        if(speed > 0 && getWristOuterLimitSwitch()) {
            System.out.println("Cannot set wrist motor speed: At upper limit");
            speed = 0;
        }
        wrist.set(TalonSRXControlMode.Current, speed);
    }

    /**
     * Sets the wrist's position.
     * @param target The target in meters.
     */
    public void setWristPosition (double target) {
        if (target < 0 || target > ArmConstants.WRIST_MAX_EXTENSION_LENGTH) {
            System.out.println("Cannot set wrist target to " + target + ": Out of range");
            stopWristMotor();
            return;
        }

        if (!wristHasBeenZeroed) {
            System.out.println("Cannot set wrist target: Has not been zeroed");
            stopWristMotor();
            return;
        }
        
        setWristVoltage(wristPID.calculate(getWristPosition(), target));
    }

    /**
     * Stops the wrist motor.
     */
    public void stopWristMotor () {
        setWristVoltage(0.0);
    }

    /**
     * Gets the arm's current position.
     * @return
     */
    public ArmPosition getArmPosition () {
        return new ArmPosition(
            getElbowPosition(),
            getWristPosition()
        );
    }
}
