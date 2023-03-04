// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ClawOpen extends CommandBase {
  Timer timer;
  boolean clawWasOpen;
  /** Creates a new ClawOpen. */
  public ClawOpen() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.claw);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    clawWasOpen = RobotContainer.claw.isOpen();
    RobotContainer.claw.open();
  }

  @Override
  public void execute(){}

  public boolean isFinished(){
    return timer.hasElapsed(0.5) || clawWasOpen;
  }

}
