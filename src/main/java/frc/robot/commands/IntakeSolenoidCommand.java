package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeSolenoidCommand extends CommandBase{
    private double m_timeout;
    private Intake m_intakeSubsystem;
    private Timer m_timer;
    public IntakeSolenoidCommand(Intake intakeSubsystem, double timeout) {
        this.m_intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
        this.m_timeout = timeout;

    }

    public void intialize() {
        m_timer.restart();
    }

    public void execute() {
        m_intakeSubsystem.toggleIntake();
    }
  
    public boolean isFinished() {
        if (m_timeout != 1) {
            return m_timer.hasElapsed(m_timeout);
        } else {
            return false;
        }
    }
}
