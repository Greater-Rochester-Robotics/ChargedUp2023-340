// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TargetConstants;

public class Target extends SubsystemBase {
  int grid;
  int col;
  int row;
  public GoalLocation[][][] goalLocations;
  public GoalLocation[][][] redGoalLocations;
  ShuffleboardTab tab;

  /** Creates a new Target. */
  public Target() {
    // MjpegServer serverStream = new MjpegServer("Anything", 1181);
    // RawSource source = new RawSource("Source", PixelFormat.kRGB565, 10, 10, 1);
    // CvSource source2 = new CvSource("Source2", PixelFormat.kRGB565, 10, 10, 1);
    // Mat mat = ;
    // source2.putFrame();
    // serverStream.setSource(source);
    tab = Shuffleboard.getTab("Tab Title");
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        for(int k = 0; k < 3; k++) {
          int ii = i;
          int jj = j;
          int kk = k;
          tab.addBoolean("Boolean G"+ii+"C"+jj+"R"+kk, () -> this.isTarget(ii, jj, kk)).withPosition(i * 3 + j, k);
        }
      }
    }

    goalLocations = new GoalLocation[][][] {
      {
        {
          new GoalLocation(new Translation2d(1.85, .5), TargetConstants.HIGH_POLE),
          new GoalLocation(new Translation2d(1.85, .5), TargetConstants.MED_POLE),
          new GoalLocation(new Translation2d(1.85, .5), TargetConstants.LOW_POLE)
        },
        {
          new GoalLocation(new Translation2d(1.85, 1.07), TargetConstants.HIGH_POLE),
          new GoalLocation(new Translation2d(1.85, 1.07), TargetConstants.MED_POLE),
          new GoalLocation(new Translation2d(1.85, 1.07), TargetConstants.LOW_POLE)
        },
        {
          new GoalLocation(new Translation2d(1.85, 1.63), TargetConstants.HIGH_POLE),
          new GoalLocation(new Translation2d(1.85, 1.63), TargetConstants.MED_POLE),
          new GoalLocation(new Translation2d(1.85, 1.63), TargetConstants.LOW_POLE)       
         }
      },
      {
        {
          new GoalLocation(new Translation2d(1.85, 2.16), TargetConstants.HIGH_POLE),
          new GoalLocation(new Translation2d(1.85, 2.16), TargetConstants.MED_POLE),
          new GoalLocation(new Translation2d(1.85, 2.16), TargetConstants.LOW_POLE)        
        },
        {
          new GoalLocation(new Translation2d(1.85, 2.68), TargetConstants.HIGH_POLE),
          new GoalLocation(new Translation2d(1.85, 2.68), TargetConstants.MED_POLE),
          new GoalLocation(new Translation2d(1.85, 2.68), TargetConstants.LOW_POLE)       
        },
        {
          new GoalLocation(new Translation2d(1.85, 3.28), TargetConstants.HIGH_POLE),
          new GoalLocation(new Translation2d(1.85, 3.28), TargetConstants.MED_POLE),
          new GoalLocation(new Translation2d(1.85, 3.28), TargetConstants.LOW_POLE)        
        }
      },
      {
        {
          new GoalLocation(new Translation2d(1.85, 3.88), TargetConstants.HIGH_POLE),
          new GoalLocation(new Translation2d(1.85, 3.88), TargetConstants.MED_POLE),
          new GoalLocation(new Translation2d(1.85, 3.88), TargetConstants.LOW_POLE)        
        },
        {
          new GoalLocation(new Translation2d(1.85, 4.39), TargetConstants.HIGH_POLE),
          new GoalLocation(new Translation2d(1.85, 4.39), TargetConstants.MED_POLE),
          new GoalLocation(new Translation2d(1.85, 4.39), TargetConstants.LOW_POLE)        
        },
        {
          new GoalLocation(new Translation2d(1.85, 4.97), TargetConstants.HIGH_POLE),
          new GoalLocation(new Translation2d(1.85, 4.97), TargetConstants.MED_POLE),
          new GoalLocation(new Translation2d(1.85, 4.97), TargetConstants.LOW_POLE)        
        }
      }
    };
    redGoalLocations = new GoalLocations[3][3][3];
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        for(int k = 0; k < 3; k++) {
          redGoalLocations[i][j][k] = new GoalLocation(new Translation2d(Constants.TargetConstants.FIELD_WIDTH - goalLocations[i][j][k].getX(),goalLocations[i][j][k].getY()), goalLocations[i][j][k].getHeight());
        }

      }

    }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Target Grid", grid);
    SmartDashboard.putNumber("Target Column", col);
    SmartDashboard.putNumber("Target Row", row);
    SmartDashboard.putNumber("Target X", getTargetPosition().getX());
    SmartDashboard.putNumber("Target Y", getTargetPosition().getY());
    SmartDashboard.putNumber("Target Height", getTargetPosition().getHeight());
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
      return goalLocations[grid][col][row];
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

  public class GoalLocation {
    private Translation2d position;
    private double height;

    public GoalLocation(Translation2d position, double height) {
      this.position = position;
      this.height = height;
    }

    public double getX() {
      return position.getX();
    }

    public double getY() {
      return position.getY();
    }

    public double getHeight() {
      return height;
    }
  }
}
