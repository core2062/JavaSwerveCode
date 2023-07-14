package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DelayCommand extends CommandBase {
    private double m_timeout;
    private Timer m_timer = new Timer();
    public DelayCommand(double timeout) {
        this.m_timeout = timeout;
    }
    
    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
    }
    
    public void end(boolean interuppted) {
        m_timer.stop();
    }
    
    @Override
    public boolean isFinished() {
        System.out.println(m_timer.get());
        if (m_timeout != -1) {
            return m_timer.hasElapsed(m_timeout);
        } else {
            return false;
        }
    }    
}
