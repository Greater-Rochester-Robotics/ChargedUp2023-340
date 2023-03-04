// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Target.GoalLocation;

public class DriveToTarget extends CommandBase {
  double currentAngle;
  Pose2d currentPose;
  double angle;
  GoalLocation currentTarget;
  Alliance currentAlliance;
  int onTargetCount = 0;
  int[] target;
  Translation2d targetPose;
  Timer timer;
  TrapezoidProfile.Constraints constraints;
  TrapezoidProfile forwardProfile;
  TrapezoidProfile strafeProfile;
  PIDController forwardController;
  PIDController strafeController;
  Pose2d intpPose2d;
  double forwardDistance;
  double strafeDistance;
  double curForwardDistance;
  double curStrafeDistance;

  /** Creates a new DriveToTarget. */
  public DriveToTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);
    timer = new Timer();
    //makes a PID controller and initializes constraints, a goal position and an initial position
    forwardController = new PIDController(Constants.SwerveDriveConstants.DRIVE_POS_ERROR_CONTROLLER_P, Constants.SwerveDriveConstants.DRIVE_POS_ERROR_CONTROLLER_I, Constants.SwerveDriveConstants.DRIVE_POS_ERROR_CONTROLLER_D);
    strafeController = new PIDController(Constants.SwerveDriveConstants.DRIVE_POS_ERROR_CONTROLLER_P, Constants.SwerveDriveConstants.DRIVE_POS_ERROR_CONTROLLER_I, Constants.SwerveDriveConstants.DRIVE_POS_ERROR_CONTROLLER_D);
    constraints = new TrapezoidProfile.Constraints(Constants.SwerveDriveConstants.MOTOR_MAXIMUM_VELOCITY, Constants.SwerveDriveConstants.MAXIMUM_ACCELERATION);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentAngle = RobotContainer.swerveDrive.getGyroInDegYaw();
    currentPose = RobotContainer.swerveDrive.getCurPose2d();
    currentTarget = RobotContainer.target.getTargetPosition();
    currentAlliance = DriverStation.getAlliance();
    target = RobotContainer.target.getTarget();
    targetPose = RobotContainer.target.getTargetPosition().getPosition();
    forwardDistance = targetPose.getX() - currentPose.getX();
    strafeDistance = targetPose.getY() - currentPose.getY();
    // column = 0 or 2, row = 2
    if ((target[1] == 0 || target[1] == 2) && target[2] == 0) {
      angle = Math.round(currentAngle / Constants.TWO_PI) * Constants.TWO_PI;
    } else {
      angle = Math.round(currentAngle / Math.PI) * Math.PI;
    }

    TrapezoidProfile.State forwardGoal = new TrapezoidProfile.State(targetPose.getX(), 0);
    TrapezoidProfile.State forwardInitial = new TrapezoidProfile.State(currentPose.getX(), 0);
    TrapezoidProfile.State strafeGoal = new TrapezoidProfile.State(targetPose.getY(), 0);
    TrapezoidProfile.State strafeInitial = new TrapezoidProfile.State(currentPose.getY(), 0);
    //Constructs a TrapezoidProfile
    forwardProfile = new TrapezoidProfile(constraints, forwardGoal, forwardInitial);
    strafeProfile = new TrapezoidProfile(constraints, strafeGoal, strafeInitial);
    intpPose2d = RobotContainer.swerveDrive.getCurPose2d();
    curForwardDistance = 0.0;
    curStrafeDistance = 0.0;
    forwardController.reset();
    strafeController.reset();
    timer.reset();
    timer.start();
    curForwardDistance = forwardDistance;
    curStrafeDistance = strafeDistance;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TrapezoidProfile.State forwardTargetState = forwardProfile.calculate(timer.get());
    TrapezoidProfile.State strafeTargetState = strafeProfile.calculate(timer.get());
    curForwardDistance = intpPose2d.getTranslation().getDistance(RobotContainer.swerveDrive.getCurPose2d().getTranslation());
    curStrafeDistance = intpPose2d.getTranslation().getDistance(RobotContainer.swerveDrive.getCurPose2d().getTranslation());
    double forwardOutput = forwardTargetState.velocity + forwardController.calculate(curForwardDistance, forwardDistance);
    double strafeOutput = strafeTargetState.velocity + strafeController.calculate(curForwardDistance, strafeDistance);
    double rotOutput = RobotContainer.swerveDrive.getRobotRotationPIDOut(angle);
    // System.out.println("pid output"+output);
    //Turns the robot
    RobotContainer.swerveDrive.driveRobotCentric(forwardOutput, strafeOutput, rotOutput, true, false);
    //If the robot is facing the target within a certain tolerance than it increases the onTargetCount if not it sets the onTargetCount to 0
    if(Math.abs(angle - RobotContainer.swerveDrive.getGyroInRadYaw()) < .03 ){
      onTargetCount++;
    }else{
      onTargetCount = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(forwardDistance - curForwardDistance) <= 0.05 && Math.abs(strafeDistance - curStrafeDistance) <= 0.05 && onTargetCount >= 10;
  }
}
