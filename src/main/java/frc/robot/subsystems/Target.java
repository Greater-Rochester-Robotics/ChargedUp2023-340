// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Target extends SubsystemBase {
  int grid;
  int col;
  int row;
  GoalLocation[][][] blueLocations;
  GoalLocation[][][] redLocations;

  /** Creates a new Target. */
  public Target() {
    blueLocations = new GoalLocation[][][] {
      {
        {
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0)
        },
        {
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0)
        }, 
        {
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0)
        }
      }, 
      {
        {
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0)
        }, 
        {
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0)
        }, 
        {
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0)
        }
      }, 
      {
        {
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0)
        }, 
        {
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0)
        }, 
        {
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0)
        }
      }
    };

    redLocations = new GoalLocation[][][] {
      {
        {
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0)
        },
        {
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0)
        }, 
        {
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0)
        }
      }, 
      {
        {
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0)
        }, 
        {
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0)
        }, 
        {
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0)
        }
      }, 
      {
        {
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0)
        }, 
        {
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0)
        }, 
        {
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0), 
          new GoalLocation(new Pose2d(), 0.0)
        }
      }
    };
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public int[] getTarget() {
    return new int[]{grid, col, row};
  }

  public void setTarget(int grid, int col, int row) {
    this.grid = grid;
    this.col = col;
    this.row = row;
  }

  public GoalLocation getTargetPosition() {
    if(DriverStation.getAlliance() == Alliance.Blue) {
      return blueLocations[grid][col][row];
    } else {
      return redLocations[grid][col][row];
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
    private Pose2d position;
    private double height;

    public GoalLocation(Pose2d position, double height) {
      this.position = position;
      this.height = height;
    }
  }
}
