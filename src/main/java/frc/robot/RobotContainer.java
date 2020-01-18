/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.CloseGate;
import frc.robot.commands.DefaultAutoCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PnuematicSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem drive = new DriveSubsystem();
  private final PnuematicSubsystem pneumatics = new PnuematicSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final LEDSubsystem led = new LEDSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();

  public static XboxController driverStick = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
  public static XboxController operatorStick = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);


  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    autoChooser.addOption("Default", new DefaultAutoCommand(drive));
    //autoChooser.addOption("Test Auto", new TestAuto());
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(operatorStick, Constants.A_BUTTON).whenPressed(new CloseGate(pneumatics));
    new JoystickButton(driverStick, Constants.A_BUTTON).whileHeld(new AutoAlign(drive, vision));
  }

  public LEDSubsystem getLED() {
    return led;
  }

  public DriveSubsystem getDrive() {
    return drive;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
