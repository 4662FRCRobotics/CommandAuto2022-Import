// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveNormSubsystem;

public class DriveDistanceSMPID extends CommandBase {

  private double m_dDistance;
  //private boolean m_bIsDashboard;
  private DriveNormSubsystem m_drive;
  
  /** Creates a new AutoDriveDistance. */
  public DriveDistanceSMPID(double distance, DriveNormSubsystem drive) {
    m_drive = drive; 
    m_dDistance = distance;
    //m_bIsDashboard = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  /*public DriveDistanceSMPID(DriveNormSubsystem drive) {
    m_drive = drive;
    m_dDistance = m_drive.getDashboardDistance();
    //m_dRotation = 0;
    m_bIsDashboard = true;
    addRequirements(m_drive);
  }*/

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*if (m_bIsDashboard) {
      m_dDistance = m_drive.getDashboardDistance();
      m_drive.resetPIDDriveController();
    }*/
    m_drive.initDriveController(m_dDistance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_drive.execDriveController();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.endDriveController();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.isDriveAtSetpoint();
  }
}
