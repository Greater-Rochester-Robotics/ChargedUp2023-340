// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

/**
 * This command sets the current position of the shoulders as zero
 * degrees. This command can be run while the robot is disabled.
 * Must wait 10 seconds for command to run
 */
public class ArmResetShoulderToZero extends CommandBase {
  Timer timer = new Timer();
  /**
   * Creates a new ArmResetLeftShoulderToZero.
   */
  public ArmResetShoulderToZero() {
    addRequirements(RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Call the reset function to zero all of the modules 
    timer.reset();
    timer.start();
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    if(!interrupted && DriverStation.isDisabled()){
      RobotContainer.arm.zeroBothShoulder();
    }else if(interrupted){
      System.out.println("WARNING: RESET OF SHOULDER COMMAND INTERRUPTED - RESET FAILED");
      DriverStation.reportWarning("WARNING: RESET OF SHOULDER COMMAND INTERRUPTED - RESET FAILED",false);
    }else if(!DriverStation.isDisabled()){
      System.out.println("WARNING: ROBOT NOT DISABLED (DISABLE ROBOT!) - RESET FAILED");
      DriverStation.reportWarning("WARNING: ROBOT NOT DISABLED (DISABLE ROBOT!) - RESET FAILED",false);
    }
  }

  @Override
  public boolean isFinished() {
    return (timer.get() >= 10) || !DriverStation.isDisabled();
  }

  public boolean runsWhenDisabled(){
    return true;
  }
}
