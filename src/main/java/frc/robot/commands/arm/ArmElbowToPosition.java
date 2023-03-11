// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class ArmElbowToPosition extends CommandBase {
  double position;
  int onTarget;
  double maxVelReached;
  /** Creates a new ArmElbowToPosition. */
  public ArmElbowToPosition(double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    onTarget = 0;
    maxVelReached = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(Math.abs(RobotContainer.arm.getElbowVelocity()) > maxVelReached){
    //   maxVelReached = Math.abs(RobotContainer.arm.getElbowVelocity());
    //   SmartDashboard.putNumber("elbow max velocity reached", maxVelReached);
    // }
    // System.out.print("angle: " + Math.round(Units.radiansToDegrees(RobotContainer.arm.getElbowPosition()))+"  ");
    RobotContainer.arm.setElbowPosition(position);
    if(Math.abs(RobotContainer.arm.getElbowPosition()-position) < ArmConstants.ELBOW_CLOSED_LOOP_ERROR){
      onTarget++;
      // System.out.println("ON Target");
    }else{
      onTarget = 0;
      // System.out.println("OFF Target");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.arm.stopElbow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  onTarget >= 10 || Math.abs(RobotContainer.arm.getElbowPosition()) > ArmConstants.MAX_ELBOW_ANGLE;
  }
}
