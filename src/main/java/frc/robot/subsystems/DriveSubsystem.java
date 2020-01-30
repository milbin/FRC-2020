/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax frontLeftDrive = new CANSparkMax(Constants.FRONT_LEFT_DRIVE_CAN, MotorType.kBrushless);
  private CANSparkMax rearLeftDrive = new CANSparkMax(Constants.REAR_LEFT_DRIVE_CAN, MotorType.kBrushless);
  private CANSparkMax frontRightDrive = new CANSparkMax(Constants.FRONT_RIGHT_DRIVE_CAN, MotorType.kBrushless);
  private CANSparkMax rearRightDrive = new CANSparkMax(Constants.REAR_RIGHT_DRIVE_CAN, MotorType.kBrushless);
  
  private DifferentialDrive drive = new DifferentialDrive(frontLeftDrive, frontRightDrive);

  private AHRS navx = new AHRS(SerialPort.Port.kMXP);

  private CANEncoder leftDriveEnc = frontLeftDrive.getEncoder();
  private CANEncoder rightDriveEnc = frontRightDrive.getEncoder();

  private DifferentialDriveOdometry odometry;

  public DriveSubsystem() {
    // motors are automatically inverted in the DifferentialDrive class
    rearLeftDrive.follow(frontLeftDrive);
    rearRightDrive.follow(frontRightDrive);
    resetEnc();
    zeroHeading();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  public void drive(double throttle, double yaw) {
    drive.arcadeDrive(throttle, yaw);
  }

  public void voltageDrive(double leftVolts, double rightVolts) {
    frontLeftDrive.setVoltage(leftVolts);
    frontRightDrive.setVoltage(-rightVolts);
    drive.feed();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncRate(), getRightEncRate());
  }


  public double getLeftEncPos() {
    return leftDriveEnc.getPosition();
  }

  public double getRightEncPos() {
    return rightDriveEnc.getPosition();
  }

  public double getLeftEncRate() {
    return leftDriveEnc.getVelocity() / 60 * Constants.FEET_PER_ROTATIONS;
  }

  public double getRightEncRate() {
    return rightDriveEnc.getVelocity() / 60 * Constants.FEET_PER_ROTATIONS;
  }

  public double getLeftDistance() { // returns distance in feet
    return getLeftEncPos() * Constants.FEET_PER_ROTATIONS;
  }

  public double getRightDistance() {
    return getRightEncPos() * Constants.FEET_PER_ROTATIONS;
  }

  public double getHeading() {
    return Math.IEEEremainder(navx.getAngle(), 360) * (Constants.GYRO_REVERSED ? -1.0 : 1.0);
  }

  public double getTurnRate() {
    return navx.getRate();
  }

  public void resetEnc() {
    leftDriveEnc.setPosition(0.0);
    rightDriveEnc.setPosition(0.0);
  }

  public void zeroHeading() {
    navx.reset();
  }

  public void resetOdometry(Pose2d pose) {
    resetEnc();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void stop() {
    drive.arcadeDrive(0.0, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDistance(), getRightDistance());
    SmartDashboard.putNumber("Left Distance", getLeftDistance());
    SmartDashboard.putNumber("Right Distance", getRightDistance());
    SmartDashboard.putNumber("Heading", getHeading());
  }
}
