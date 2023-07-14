package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;

public class BalanceCommand extends CommandBase {
    private Swerve s_Swerve;
    private PIDController m_balancePIDController;
    private int m_stage;
    private boolean isCommandFinished;
    private double kP, kI, kD;

    public BalanceCommand(Swerve s_Swerve, int stage) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.m_stage = stage;
        this.m_balancePIDController = new PIDController(0.0,0.0,0.0);
        this.isCommandFinished = false;
        
        kP = 0.015;
        kI = 0.0;
        kD = 0.0;
    }

    @Override
    public void initialize() {
        m_balancePIDController.setPID(kP, kI, kD);;
        m_balancePIDController.setSetpoint(0.0);
        if (m_stage == 1)
        {
            m_balancePIDController.setTolerance(4.0);
            isCommandFinished = false;
        }
        else if(m_stage == 2) 
        {
            m_balancePIDController.setTolerance(1);
            isCommandFinished = false;
        } 
        else if(m_stage == 0) 
        {
            isCommandFinished = true;
        }
    }



    @Override
    public void execute() {
        double currPitch = s_Swerve.getPitch();
        double currRoll = s_Swerve.getRoll();
        double motorSpeedPitch = m_balancePIDController.calculate(currPitch);
        double motorSpeedRoll = m_balancePIDController.calculate(currRoll);

        if (m_stage == 1)
        {
            /* Drive */
            // System.out.println("Stage One");
            if (motorSpeedRoll < 0) {
                // System.out.println("Stage One");
                s_Swerve.drive(
                new Translation2d(-0.45, 0.0).times(Constants.Swerve.autoBalanceMaxSpeed), 0.0, 
                true, true);
            } else if (motorSpeedRoll > 0.6) {
                // System.out.println("Stage One");
                s_Swerve.drive(
                new Translation2d(0.45, 0.0).times(Constants.Swerve.autoBalanceMaxSpeed), 0.0, 
                true, true);
            }
            
            
        }
        else if (m_stage == 2)
        {
            if (motorSpeedRoll > 0) {
                motorSpeedRoll += 0.02;
            } 
            else if (motorSpeedRoll < 0)
            {
                motorSpeedRoll -= 0.02 ;
            }
            else if (motorSpeedPitch > 0) {
                motorSpeedPitch += 0.02;
            } 
            else if (motorSpeedPitch < 0)
            {
                motorSpeedPitch -= 0.02 ;
            }
            /* Drive */
            if (motorSpeedRoll < 0) {
                System.out.println("Stage Two");
                s_Swerve.drive(
                new Translation2d(-0.2, 0.0).times(Constants.Swerve.autoBalanceMaxSpeed), 0.0, 
                true, true);
            } else if (motorSpeedRoll > 0) {
                System.out.println("Stage Two");
                s_Swerve.drive(
                new Translation2d(0.2, 0.0).times(Constants.Swerve.autoBalanceMaxSpeed), 0.0, 
                true, true);
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (isCommandFinished == true || m_balancePIDController.atSetpoint()) {
            return true;
        } else
        {
            return m_balancePIDController.atSetpoint();
        }
    }
}
