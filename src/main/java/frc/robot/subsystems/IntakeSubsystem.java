/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private WPI_VictorSPX conveyor = new WPI_VictorSPX(Constants.CONVEYOR_CAN_ID);
  private WPI_VictorSPX intake = new WPI_VictorSPX(Constants.INTAKE_CAN_ID);

  // add sensors for ball detection

  private int cellCount = 3; // can preload 3 cells

  public IntakeSubsystem() {}

  public void setElevator() {
    conveyor.set(0.75);
    intake.set(0.75);
  }

  public void stop() {
    conveyor.set(0.0);
    intake.set(0.0);
  }

  public void cellIn() {
    cellCount++;
  }

  public void cellOut() {
    cellCount--;
  }

  public int getCellCount() {
    return cellCount;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (!intakeCellCounter.get()) { // .get() returns true if the circuit is open
    //   cellIn();
    // }
    // if (!shooterCellCounter.get()) {
    //   cellOut();
    // }
  }
}
