/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PnuematicSubsystem extends SubsystemBase {
  private Compressor compressor = new Compressor();

  private DoubleSolenoid intake = new DoubleSolenoid(Constants.INTAKE_SOLENOID_FORWARD_PORT, Constants.INTAKE_SOLENOID_REVERSE_PORT);
  private DoubleSolenoid ballRaiseMechanism = new DoubleSolenoid(Constants.BALL_RAISE_SOLENOID_FORWARD_PORT, Constants.BALL_RAISE_SOLENOID_REVERSE_PORT);
  private Solenoid intakeCover = new Solenoid(Constants.INTAKE_COVER_SOLENOID_PORT);
  private Solenoid gate = new Solenoid(Constants.GATE_SOLENOID_PORT);
  private Solenoid arm = new Solenoid(Constants.ARM_SOLENOID_PORT);

  private boolean isIntakeDown = false;
  private boolean isGateOpen = false;
  private boolean isArmDown = false;

  public PnuematicSubsystem() {
  }

  public void startCompressor() {
    compressor.start();
  }

  public void stopCompressor() {
    compressor.stop();
  }

  public boolean isIntakeDown() {
    return isIntakeDown;
  }

  public boolean isGateOpen() {
    return isGateOpen;
  }

  public boolean isArmDown() {
    return isArmDown;
  }

  public void moveIntakeDown() {
    if (!isIntakeDown) {
      isIntakeDown = true;
      intake.set(Value.kForward); // test
    }
  }

  public void moveIntakeUp() {
    if (isIntakeDown) {
      isIntakeDown = false;
      intake.set(Value.kReverse); // test
    }
  }

  public void openGate() {
    if (!isGateOpen) {
      isGateOpen = true;
      gate.set(true);
    }
  }

  public void closeGate() {
    if (isGateOpen) {
      isGateOpen = false;
      gate.set(false);
    }
  }

  public void moveArmDown() {
    if (!isArmDown) {
      isArmDown = true;
      arm.set(true);
    }
  }

  public void moveArmUp() {
    if (isArmDown) {
      isArmDown = false;
      arm.set(false);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Gate Status:", isGateOpen()); // light indicator
    SmartDashboard.putBoolean("Arm Status:", isArmDown());
    SmartDashboard.putBoolean("Intake Status:", isIntakeDown());
  }
}
