// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class ArmWristExtend extends CommandBase {
  Timer timer = new Timer();
  boolean waitForExtend;
  
  public ArmWristExtend(){
    this(true);
  }

  public ArmWristExtend(boolean waitForExtend) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
    this.waitForExtend = waitForExtend;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.arm.extendWrist();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !waitForExtend || timer.hasElapsed(ArmConstants.WRIST_EXTENSION_DELAY);
  }
}
