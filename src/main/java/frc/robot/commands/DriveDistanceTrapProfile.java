// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants.DriveNormSubsystemConst;
import frc.robot.subsystems.DriveNormSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveDistanceTrapProfile extends TrapezoidProfileCommand {
  /** Creates a new DriveDistanceTrapProfile. */

  public DriveDistanceTrapProfile(double distMeter, DriveNormSubsystem driveNorm) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
              DriveNormSubsystemConst.kMAX_SPEED_METERS_PER_SECOND,
              DriveNormSubsystemConst.kMAX_ACCELERATION_METERS_PER_SECOND_SQUARED
            ),
            // Goal state
            new TrapezoidProfile.State(distMeter, 0)),
        // pipe the profile state to the drive
        setpointState -> driveNorm.setDriveStates(setpointState, setpointState),
          // require drive
          driveNorm
        );

    driveNorm.resetTrapezoidal();
  }
}
