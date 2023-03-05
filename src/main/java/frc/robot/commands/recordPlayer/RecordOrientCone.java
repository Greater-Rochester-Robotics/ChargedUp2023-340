// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.recordPlayer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.RecordPlayerConstants;

public class RecordOrientCone extends CommandBase {
  boolean sensor0;
  boolean sensor1;
  boolean previousSensor0;
  boolean hasFoundPosition;
  double goalPosition;
  /** Creates a new OrientCone. */
  public RecordOrientCone() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.recordPlayer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.recordPlayer.setRotationMotor(RecordPlayerConstants.ROTATE_MOTOR_SPEED); //TODO: set the speed
    previousSensor0 = RobotContainer.recordPlayer.getConePositionSensor0();
    hasFoundPosition = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sensor0 = RobotContainer.recordPlayer.getConePositionSensor0();
    sensor1 = RobotContainer.recordPlayer.getConePositionSensor1();

    //false, false -> true false = stop
    //true, false -> true, true = keep going
    //true true -> false true = stop (favored)
    //false true -> true, true = keep going
    //true, true -> true false = stop

    if(sensor1 && !sensor0  && previousSensor0 && !hasFoundPosition) {
      goalPosition = RobotContainer.recordPlayer.getEncoderAngle() + 0.25;
      hasFoundPosition = true;
    }

    if(hasFoundPosition){
      RobotContainer.recordPlayer.rotateToAngle(goalPosition);
    }

    previousSensor0 = sensor0;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.recordPlayer.stopRotationMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasFoundPosition && Math.abs(goalPosition - RobotContainer.recordPlayer.getEncoderAngle()) < 0.1;
  }
}
