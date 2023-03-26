// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

// TODO: Finalize and test.

public class ArmWristHome extends CommandBase {
    boolean override;

    public ArmWristHome () {
        this(false);
    }

    public ArmWristHome (boolean override) {
        addRequirements(RobotContainer.arm);
        this.override = override;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize () {
        if(override){
            System.out.println("Forcing wrist to home");
        }else{
            System.out.println("Wrist homing without override");
        }
        System.out.println("Starting homing wrist from: " + RobotContainer.arm.getWristPosition());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute () {
        if(override || !RobotContainer.arm.getWristBeenZeroed() ){
            RobotContainer.arm.setWristDutyCycle(-ArmConstants.WRIST_HOMING_SPEED);
        }else{
            RobotContainer.arm.stopWristMotor();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end (boolean interrupted) {
        RobotContainer.arm.stopWristMotor();
        if(RobotContainer.arm.getWristInnerLimitSwitch()){
            RobotContainer.arm.zeroWrist();
            System.out.println("Wrist has fully homed");
        }else if(!override && RobotContainer.arm.getWristBeenZeroed()){
            System.out.println("Wrist was previously homed and was not overridden");
        }else{
            System.out.println("Wrist has not fully homed, but home command ended is interrupt:" + interrupted);
        }
        System.out.println("Homing ended with wrist at: " + RobotContainer.arm.getWristPosition());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished () {
        return (RobotContainer.arm.getWristBeenZeroed() && !override) || 
            RobotContainer.arm.getWristInnerLimitSwitch();
    }
}
