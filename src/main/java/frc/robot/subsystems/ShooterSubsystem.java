/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  
  private CANSparkMax shooterLeft = new CANSparkMax(Constants.SHOOTER_LEFT_CAN_ID, MotorType.kBrushless);
  private CANSparkMax shooterRight = new CANSparkMax(Constants.SHOOTER_RIGHT_CAN_ID, MotorType.kBrushless);

  private CANEncoder leftEnc = new CANEncoder(shooterLeft);

  private CANPIDController pid;
  private double maxRPM = 5500;
  private double kP = 1.0;
  private double target;


  public ShooterSubsystem() {
    shooterLeft.setInverted(true); // left side needs to turn counter clockwise
    shooterRight.follow(shooterLeft);
    pid = shooterLeft.getPIDController();
    pid.setP(kP);
    pid.setI(0.0);
    pid.setD(0.0);
    pid.setIZone(0.0);
    pid.setFF(0.0);
    pid.setOutputRange(-1, 1);
  }

  public void spinUp(double targetRPM) {
    target = targetRPM;
    if (targetRPM > maxRPM) {
      targetRPM = maxRPM;
    }
    pid.setReference(targetRPM, ControlType.kVelocity);
  }

  public void stop() {
    shooterLeft.set(0.0);
    shooterRight.set(0.0);
  }
  
  public double getVelocity() {
    return leftEnc.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM", getVelocity());
    SmartDashboard.putBoolean("Shooter At Speed?", target == getVelocity());
  }
}
