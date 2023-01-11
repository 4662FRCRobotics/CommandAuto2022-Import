package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.DriveNormSubsystem;

public class AutoControl extends CommandBase {
    
    private Command m_currentCommand;
    private Autonomous m_autonomous;
    private DriveNormSubsystem m_driveNorm;

    public AutoControl(Autonomous autonomous, DriveNormSubsystem driveNorm) {
        m_autonomous = autonomous;
        m_driveNorm = driveNorm;

        addRequirements(m_autonomous, m_driveNorm);  
    }

    @Override
    public void initialize() {
        // getInstance();
        m_autonomous.initGetCommand();
        m_currentCommand = m_autonomous.getNextCommand();
        m_currentCommand.initialize();
    }

    @Override
    public void execute() {
        m_currentCommand.execute();
       //  System.out.println("execute");
    }

    @Override
    public void end(boolean interrupted) {
        m_currentCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        boolean areWeThereYet = true;
        Command nextCommand = null;
        if (m_currentCommand.isFinished() == false) {
            areWeThereYet = false;
        } else {
            nextCommand = m_autonomous.getNextCommand();
            if (nextCommand != null) {
                switchCommand(nextCommand);
                areWeThereYet = false;
            }
        }
        return areWeThereYet;
    }

    // stops current command then goes to next one
    private void switchCommand(final Command cmd) {
        m_currentCommand.end(false);
        m_currentCommand = cmd;
        m_currentCommand.initialize();
    }

}
