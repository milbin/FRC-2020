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
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class JoystickDrive extends CommandBase {

  private DriveSubsystem driveSub;

  public JoystickDrive(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveSub = drive;
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = -RobotContainer.driverStick.getY(Hand.kLeft);
    double yaw = RobotContainer.driverStick.getX(Hand.kRight);
    if (yaw < Constants.DEADBAND) {
      yaw = 0.0;
    }
    if (throttle < Constants.DEADBAND) {
      throttle = 0.0;
    }
    driveSub.drive(throttle, yaw);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
