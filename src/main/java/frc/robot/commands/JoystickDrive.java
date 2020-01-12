/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class JoystickDrive extends CommandBase {
  /**
   * Creates a new JoystickDrive.
   */
  public JoystickDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = -Robot.robotContainer.driverStick.getY(Hand.kLeft);
    double yaw = Robot.robotContainer.driverStick.getX(Hand.kRight);
    if (yaw < Constants.DEADBAND) {
      yaw = 0.0;
    }
    if (throttle < Constants.DEADBAND) {
      throttle = 0.0;
    }
    Robot.drive.drive(throttle, yaw);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.drive.stop();
    Robot.drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
