// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.Random;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GenRandomBoolean extends CommandBase {
  /** Creates a new GenRandomBoolean. */

  private final BooleanConsumer m_output;
  private final Random randBool = new Random();

  public GenRandomBoolean(BooleanConsumer outputBool) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_output = requireNonNullParam(outputBool,"outputBool", "GenRandomBoolean");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_output.accept(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_output.accept(randBool.nextBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
