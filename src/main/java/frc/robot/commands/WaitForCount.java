package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;

public class WaitForCount extends CommandBase {
  protected Timer m_timer = new Timer();
  private final double m_duration;
  IntSupplier m_isWaitCount;
  int m_iWaitCount;

  /**
   * Creates a variation on WPILIB WaitCommand. This command will do nothing, and
   * end after the specified duration * count.
   *
   */

  public WaitForCount(double seconds, IntSupplier waitCount) {
    m_isWaitCount = waitCount;
    m_duration = seconds;
    SendableRegistry.setName(this, getName() + ": " + seconds + " seconds");
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_iWaitCount = m_isWaitCount.getAsInt();
    m_iWaitCount--;
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    boolean isTimeUp = m_timer.hasElapsed(m_duration);
    if (m_iWaitCount < 0) {
      isTimeUp = true;
    }

    if (isTimeUp) {
      if (m_iWaitCount > 0) {
        isTimeUp = false;
        end(false);
        m_timer.reset();
        m_timer.start();
        m_iWaitCount--;
      }
    }
    return isTimeUp;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
