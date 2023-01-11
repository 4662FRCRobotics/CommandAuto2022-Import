// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ControllerConst;
import frc.robot.commands.*;
import frc.robot.libraries.ConsoleAuto;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.DriveNormSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Joystick m_joystick = new Joystick(ControllerConst.kDRIVER_JOYSTICK_PORT);
  private final ConsoleAuto m_consoleAuto = new ConsoleAuto(ControllerConst.kAUTONOMOUS_CONSOLE_PORT);
  private final DriveNormSubsystem m_driveNorm = new DriveNormSubsystem();
  
  private final Autonomous m_autonomous = new Autonomous(m_consoleAuto, m_driveNorm);

  private final AutoSelect m_autoSelect = new AutoSelect(m_autonomous);
  private final AutoControl m_autoCommand = new AutoControl(m_autonomous, m_driveNorm);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /*
    m_driveNorm.setDefaultCommand(
      new ArcadeDrive(
        () -> m_joystick.getY(),
        () -> m_joystick.getX(),
        m_driveNorm)
    );
    */
    m_driveNorm.setDefaultCommand(
      Commands.run(() -> m_driveNorm.arcadeDrive(m_joystick.getY(), m_joystick.getX()), m_driveNorm)
    );
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutoSelect(){
    return m_autoSelect;
  }
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
