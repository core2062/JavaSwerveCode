package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonSRX m_intakeUpperMotor = new TalonSRX(Constants.Swerve.IntakeSolenoidConstants.kIntakeUpperMotorPort);
    private TalonSRX m_intakeLowerMotor = new TalonSRX(Constants.Swerve.IntakeSolenoidConstants.kIntakeLowerMotorPort);
    private final DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(
        PneumaticsModuleType.REVPH, 
        Constants.Swerve.IntakeSolenoidConstants.kIntakeSolenoidPorts[0], 
        Constants.Swerve.IntakeSolenoidConstants.kIntakeSolenoidPorts[1]);
    /* Controlls Intake */
    public CommandBase intakeMovementCommand() {
        System.out.println("command base");
        return this.runOnce(() -> m_intakeSolenoid.toggle());
    }

    public void toggleIntake() {
        m_intakeSolenoid.toggle();
    }

    public void setIntakeSpeed(double speed) {
        m_intakeUpperMotor.set(TalonSRXControlMode.PercentOutput, -speed);
        m_intakeLowerMotor.set(TalonSRXControlMode.PercentOutput, speed);
        System.out.println("setting intake speed");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("Dropped", () -> m_intakeSolenoid.get() == kForward, null);
    }
}
