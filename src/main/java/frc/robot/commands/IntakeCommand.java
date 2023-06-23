package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCommand extends CommandBase{
    private double m_speed, m_timeout;
    private Intake m_intakeSubsystem;
    private Timer m_timer;
    public IntakeCommand(Intake intakeSubsystem, double speed, double timeout) {
        this.m_speed = speed;
        this.m_intakeSubsystem = intakeSubsystem;
        this.m_timeout = timeout;
        addRequirements(intakeSubsystem);
    }

    public void intialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
        m_intakeSubsystem.setIntakeSpeed(m_speed);
    }

    public void end(boolean interuppted) {
        m_intakeSubsystem.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        if (m_timeout != 1) {
            return m_timer.hasElapsed(m_timeout);
        } else {
            return false;
        }
    }
}
