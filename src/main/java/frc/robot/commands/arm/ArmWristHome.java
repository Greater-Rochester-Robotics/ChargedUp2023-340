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
    public void initialize () {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute () {
        RobotContainer.arm.setWristVoltage(-ArmConstants.WRIST_HOMING_SPEED);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end (boolean interrupted) {
        RobotContainer.arm.stopWristMotor();
        if(!interrupted || !override){
            RobotContainer.arm.zeroWrist();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished () {
        return (RobotContainer.arm.getWristBeenZeroed() && !override) || RobotContainer.arm.getWristInnerLimitSwitch();
    }
}
