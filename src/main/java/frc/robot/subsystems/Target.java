// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.TargetConstants;
import frc.robot.commands.arm.ArmToPosition;

public class Target extends SubsystemBase {
  int grid;
  int col;
  int row;
  public GoalLocation[][][] blueGoalLocations;
  public GoalLocation[][][] redGoalLocations;
  ShuffleboardTab tab;
  private boolean scoring;
  // ShuffleboardTab tab;

  /** Creates a new Target. */
  public Target() {
    // MjpegServer serverStream = new MjpegServer("Anything", 1181);
    // RawSource source = new RawSource("Source", PixelFormat.kRGB565, 10, 10, 1);
    // CvSource source2 = new CvSource("Source2", PixelFormat.kRGB565, 10, 10, 1);
    // Mat mat = ;
    // source2.putFrame();
    // serverStream.setSource(source);

    //this works
    // tab = Shuffleboard.getTab("Tab Title");
    // for(int i = 0; i < 3; i++) {
    //   for(int j = 0; j < 3; j++) {
    //     for(int k = 0; k < 3; k++) {
    //       int ii = i;
    //       int jj = j;
    //       int kk = k;
    //       tab.addBoolean("Boolean G"+ii+"C"+jj+"R"+kk, () -> this.isTarget(ii, jj, kk)).withPosition(i * 3 + j, k);
    //     }
    //   }
    // }

    blueGoalLocations = new GoalLocation[][][] {
      {
        {
          new GoalLocation(new Translation2d(1.85, .5), ArmConstants.FRONT_HIGH_CONE,ArmConstants.BACK_HIGH_CONE),
          new GoalLocation(new Translation2d(1.85, .5), ArmConstants.FRONT_MIDDLE_CONE,ArmConstants.BACK_MIDDLE_CONE),
          new GoalLocation(new Translation2d(1.85, .5), ArmConstants.FRONT_LOWER_SCORE,ArmConstants.BACK_LOWER_SCORE)
        },
        {
          new GoalLocation(new Translation2d(1.85, 1.07), ArmConstants.FRONT_HIGH_CUBE,ArmConstants.BACK_HIGH_CUBE),
          new GoalLocation(new Translation2d(1.85, 1.07), ArmConstants.FRONT_MIDDLE_CUBE,ArmConstants.BACK_MIDDLE_CUBE),
          new GoalLocation(new Translation2d(1.85, 1.07), ArmConstants.FRONT_LOWER_SCORE,ArmConstants.BACK_LOWER_SCORE)
        },
        {
          new GoalLocation(new Translation2d(1.85, 1.63), ArmConstants.FRONT_HIGH_CONE,ArmConstants.BACK_HIGH_CONE),
          new GoalLocation(new Translation2d(1.85, 1.63),ArmConstants.FRONT_MIDDLE_CONE,ArmConstants.BACK_MIDDLE_CONE),
          new GoalLocation(new Translation2d(1.85, 1.63), ArmConstants.FRONT_LOWER_SCORE,ArmConstants.BACK_LOWER_SCORE)       
         }
      },
      {
        {
          new GoalLocation(new Translation2d(1.85, 2.16),ArmConstants.FRONT_HIGH_CONE,ArmConstants.BACK_HIGH_CONE),
          new GoalLocation(new Translation2d(1.85, 2.16), ArmConstants.FRONT_MIDDLE_CONE,ArmConstants.BACK_MIDDLE_CONE),
          new GoalLocation(new Translation2d(1.85, 2.16), ArmConstants.FRONT_LOWER_SCORE,ArmConstants.BACK_LOWER_SCORE)        
        },
        {
          new GoalLocation(new Translation2d(1.85, 2.68), ArmConstants.FRONT_HIGH_CUBE,ArmConstants.BACK_HIGH_CUBE),
          new GoalLocation(new Translation2d(1.85, 2.68), ArmConstants.FRONT_MIDDLE_CUBE,ArmConstants.BACK_MIDDLE_CUBE),
          new GoalLocation(new Translation2d(1.85, 2.68), ArmConstants.FRONT_LOWER_SCORE,ArmConstants.BACK_LOWER_SCORE)       
        },
        {
          new GoalLocation(new Translation2d(1.85, 3.28), ArmConstants.FRONT_HIGH_CONE,ArmConstants.BACK_HIGH_CONE),
          new GoalLocation(new Translation2d(1.85, 3.28), ArmConstants.FRONT_MIDDLE_CONE,ArmConstants.BACK_MIDDLE_CONE),
          new GoalLocation(new Translation2d(1.85, 3.28), ArmConstants.FRONT_LOWER_SCORE,ArmConstants.BACK_LOWER_SCORE)        
        }
      },
      {
        {
          new GoalLocation(new Translation2d(1.85, 3.88), ArmConstants.FRONT_HIGH_CONE,ArmConstants.BACK_HIGH_CONE),
          new GoalLocation(new Translation2d(1.85, 3.88), ArmConstants.FRONT_MIDDLE_CONE,ArmConstants.BACK_MIDDLE_CONE),
          new GoalLocation(new Translation2d(1.85, 3.88), ArmConstants.FRONT_LOWER_SCORE,ArmConstants.BACK_LOWER_SCORE)        
        },
        {
          new GoalLocation(new Translation2d(1.85, 4.39), ArmConstants.FRONT_HIGH_CUBE,ArmConstants.BACK_HIGH_CUBE),
          new GoalLocation(new Translation2d(1.85, 4.39), ArmConstants.FRONT_MIDDLE_CUBE,ArmConstants.BACK_MIDDLE_CUBE),
          new GoalLocation(new Translation2d(1.85, 4.39), ArmConstants.FRONT_LOWER_SCORE,ArmConstants.BACK_LOWER_SCORE)        
        },
        {
          new GoalLocation(new Translation2d(1.85, 4.97), ArmConstants.FRONT_HIGH_CONE,ArmConstants.BACK_HIGH_CONE),
          new GoalLocation(new Translation2d(1.85, 4.97), ArmConstants.FRONT_MIDDLE_CONE,ArmConstants.BACK_MIDDLE_CONE),
          new GoalLocation(new Translation2d(1.85, 4.97), ArmConstants.FRONT_LOWER_SCORE,ArmConstants.BACK_LOWER_SCORE)        
        }
      }
    };
    redGoalLocations = new GoalLocation[3][3][3];
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        for(int k = 0; k < 3; k++) {
          redGoalLocations[i][j][k] = new GoalLocation(
            new Translation2d(
            Constants.TargetConstants.FIELD_WIDTH - blueGoalLocations[i][j][k].getX(),
            blueGoalLocations[i][j][k].getY()), 
            blueGoalLocations[i][j][k].getFrontArmPos(),
            blueGoalLocations[i][j][k].getBackArmPos()
          );
        }
      }
    }
  }


  @Override
  public void periodic() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("/dashboard/targetting");
    table.getEntry("selection").setIntegerArray(getTargetLong());
    table.getEntry("scoring").setBoolean(scoring);
    SmartDashboard.putNumber("Target Grid", grid);
    SmartDashboard.putNumber("Target Column", col);
    SmartDashboard.putNumber("Target Row", row);
    SmartDashboard.putNumber("Target X", getTargetPosition().getX());
    SmartDashboard.putNumber("Target Y", getTargetPosition().getY());
  }

  public long[] getTargetLong() {
    return new long[]{grid, col, row};
  }
  // helper function that returns an array for a certain GoalPosition
  public int[] getTarget() {
    return new int[]{grid, col, row};
  }

  public boolean isTarget(int grid, int col, int row){
    return (grid == this.grid) && (col == this.col) && (row == this.row);
  }

  public void setTarget(int grid, int col, int row) {
    this.grid = grid;
    this.col = col;
    this.row = row;
  }

  public GoalLocation getTargetPosition() {
    if(DriverStation.getAlliance() == Alliance.Blue) {
      return blueGoalLocations[grid][col][row];
    } else { 
      return redGoalLocations[grid][col][row];
    }
  }

  public void up() {
    row--;
    if(row < 0) row = 2;
  }

  public void down() {
    row++;
    if(row > 2) row = 0;
  }

  public void right() {
    col++;
    if(col > 2) {
      next();
      col = 0;
    }
  }

  public void left() {
    col--;
    if(col < 0) {
      previous();
      col = 2;
    }
  }

  public void next() {
    grid++;
    if(grid > 2) grid = 0;
  }

  public void previous() {
    grid--;
    if(grid < 0) grid = 2;
  }

  public void setScoring(boolean scoring) {
    this.scoring = scoring;
  }

  public boolean isScoring() {
    return scoring;
  }

  public class GoalLocation {
    private Translation2d position;
    private ArmPosition armPositionBack;
    private ArmPosition armPositionFront;

    public GoalLocation(Translation2d position, ArmPosition frontArmPosition, ArmPosition backArmPosition) {
      this.position = position;
      this.armPositionFront = frontArmPosition;
      this.armPositionBack = backArmPosition;

    }

    public double getX() {
      return position.getX();
    }

    public double getY() {
      return position.getY();
    }

    public ArmPosition getFrontArmPos() {
      return armPositionFront;
    }
  
    public Command getFrontArmMoveCommand(){
      return new ArmToPosition(this.getFrontArmPos());
    }

    public ArmPosition getBackArmPos(){
      return armPositionBack;
    }

    public Command getBackArmMoveCommand(){
      return new ArmToPosition(this.getBackArmPos());
    }

    public Translation2d getPosition() {
      return position;
    }
  }
}
