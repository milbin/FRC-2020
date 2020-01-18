/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.JoystickDrive;
import frc.robot.trajectories.GatherFromTrench;
import frc.robot.trajectories.TestTrajectory;
import frc.robot.trajectories.TrenchToShoot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static RobotContainer robotContainer;
  
  public static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

  public Command autoCommand;

  public SmartDashboard sd;
  public static HashMap<String, Trajectory> paths = new HashMap<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    generateTrajectories();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    robotContainer.led.disabled();
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    autoCommand = robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
    robotContainer.led.teleopStart();
    new JoystickDrive(robotContainer.drive);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // boolean align = robotContainer.driverStick.getAButton();
    // updateLimelightTracking();

    // if (ds.getMatchTime() == 40) {
    //   led.endgameWarning();
    // }
    // if (align) {
    //   if (limelightHasTarget) {
    //     drive.drive(limelightThrottle, limelightYaw);
    //   }
    // }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  // public void updateLimelightTracking() {
  //   double tv = limelight.getEntry("tv").getDouble(0.0);
  //   double tx = limelight.getEntry("tx").getDouble(0.0);
  //   double ty = limelight.getEntry("ty").getDouble(0.0);
  //   //double ta = limelight.getEntry("ta").getDouble(0.0); // percent area of target seen

  //   if (Math.abs(tx) <= Constants.LIMELIGHT_TARGET_TOLERANCE && Math.abs(ty) <= Constants.LIMELIGHT_TARGET_TOLERANCE) {
  //     limelightHasTarget = true;
  //     led.shooterAligned();
  //     return;
  //   }

  //   if (tv < 1.0) {
  //     limelightHasTarget = false;
  //     limelightThrottle = 0.0;
  //     limelightYaw = 0.0;
  //     return;
  //   }
    
  //   limelightHasTarget = true;
  //   limelightYaw = tx * Constants.LIMELIGHT_ROTATE_KP;
  //   limelightThrottle = ty * Constants.LIMELIGHT_DRIVE_KP;

  //   if (limelightThrottle > Constants.LIMELIGHT_DRIVE_MAX_SPEED) {
  //     limelightThrottle = Constants.LIMELIGHT_DRIVE_MAX_SPEED;
  //   }

  //   if (limelightThrottle < -Constants.LIMELIGHT_DRIVE_MAX_SPEED) {
  //     limelightThrottle = -Constants.LIMELIGHT_DRIVE_MAX_SPEED;
  //   }
  // }

  public static void generateTrajectories() {
    paths.put("Test", TestTrajectory.generate());
    paths.put("Gather", GatherFromTrench.generate());
    paths.put("Travel", TrenchToShoot.generate());
    SmartDashboard.putBoolean("Paths Generated", true);
  }
}
