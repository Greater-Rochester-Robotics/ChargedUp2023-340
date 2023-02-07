// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cscore.raw.RawSource;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Target extends SubsystemBase {
  int grid;
  int col=2;
  int row;
  GoalLocation[][][] blueLocations;
  GoalLocation[][][] redLocations;
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
          tab.addBoolean("Boolean" + (i*j*k), () -> this.isTarget(ii, jj, kk)).withPosition(j * 3 + k, i);
        }
      }
    }

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
    SmartDashboard.putNumber("Target Grid", grid);
    SmartDashboard.putNumber("Target Column", col);
    SmartDashboard.putNumber("Target Row", row);
    SmartDashboard.putNumber("Target X", getTargetPosition().getX());
    SmartDashboard.putNumber("Target Y", getTargetPosition().getY());
    SmartDashboard.putNumber("Target Height", getTargetPosition().getHeight());
  }

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
