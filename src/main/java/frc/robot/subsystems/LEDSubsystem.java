/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

  private AddressableLED led = new AddressableLED(Constants.LED_PWM_PORT);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(40);

  public void init() {
    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();
    disabled();
  }

  public void disabled() {
    for (int i = 0; i < buffer.getLength(); i++) { // pattern: solid red
      buffer.setRGB(i, 255, 0, 0);
    } 
    led.setData(buffer);
  }

  public void autoRunning() {
    for (int i = 0; i < buffer.getLength(); i++) { // pattern: solid blue
      buffer.setRGB(i, 255, 0, 0);
    } 
  }

  public void teleopStart() {
    buffer.setRGB(0, 0, 0, 255);
    for (int i = 1; i < buffer.getLength(); i++) { // pattern: blue chase
      buffer.setRGB(i-1, 0, 0, 0);
      buffer.setRGB(i, 0, 0, 255);
    } 
  }

  public void endgameWarning() {
    // pattern: white strobe
  }

  public void shooterAligned() {
    // pattern: gold strobe
  }

  public void numberOfCells(int numCells) {
    for (int i = 0; i < numCells; i++) {
      
    }

  }

  public void off() {
    for (int i = 0; i < buffer.getLength(); i++) { // pattern: off
      buffer.setRGB(i, 0, 0, 0);
    } 
    led.setData(buffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
