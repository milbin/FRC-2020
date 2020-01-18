/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
  public static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

  private double tx;
  private double ty;
  private double tv;

  private double limelightYaw = 0.0;
  private double limelightThrottle = 0.0;

  public VisionSubsystem() {
    tx = limelight.getEntry("tx").getDouble(0.0);
    ty = limelight.getEntry("ty").getDouble(0.0);
    tv = limelight.getEntry("tv").getDouble(0.0);
  }

  public double getLimelightThrottle() {
    return limelightThrottle;
  }

  public double getLimelightYaw() {
    return limelightYaw;
  }

  public double getTx() {
    return tx;
  }

  public double getTy() {
    return ty;
  }

  public boolean hasTarget() {
    if (tv == 1.0) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isXAligned() {
    if (Math.abs(tx) < Constants.LIMELIGHT_TARGET_TOLERANCE) {
      return true;
    }
    return false;
  }

  public boolean isYAligned() {
    if (Math.abs(ty) < Constants.LIMELIGHT_TARGET_TOLERANCE) {
      return true;
    }
    return false;
  }

  public boolean isRobotAligned() {
    if (isXAligned() && isYAligned()) {
      return true;
    }
    return false;
  }
  

  @Override
  public void periodic() {
    tx = limelight.getEntry("tx").getDouble(0.0);
    ty = limelight.getEntry("ty").getDouble(0.0);
    tv = limelight.getEntry("tv").getDouble(0.0);

    if (!isRobotAligned() && hasTarget()) {
      limelightThrottle = ty * Constants.LIMELIGHT_DRIVE_KP;
      limelightYaw = tx * Constants.LIMELIGHT_ROTATE_KP;
    }

    if (limelightThrottle > Constants.LIMELIGHT_DRIVE_MAX_SPEED) {
      limelightThrottle = Constants.LIMELIGHT_DRIVE_MAX_SPEED;
    }

    if (limelightThrottle < -Constants.LIMELIGHT_DRIVE_MAX_SPEED) {
      limelightThrottle = -Constants.LIMELIGHT_DRIVE_MAX_SPEED;
    }
  }
}
