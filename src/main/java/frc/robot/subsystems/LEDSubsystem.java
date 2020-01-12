/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

  private Spark blinkin = new Spark(Constants.BLINKIN_PWM_PORT);

  public void disabled() {
    blinkin.set(0.61); // pattern: solid red
  }

  public void autoRunning() {
    blinkin.set(-0.09); // pattern: blue strobe
  }

  public void teleopStart() {
    blinkin.set(-0.29); // pattern: blue chase
    Timer.delay(1.0);
    blinkin.set(0.0);
  }

  public void endgameWarning() {
    blinkin.set(-0.05); // pattern: white strobe
    Timer.delay(3.0);
    blinkin.set(0.0);
  }

  public void shooterAligned() {
    blinkin.set(-0.07); // pattern: gold strobe
    Timer.delay(0.5);
    blinkin.set(0.0);
  }

  public void numberOfCells(int numCells) {
    for (int i = 0; i < numCells; i++) {
      
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
