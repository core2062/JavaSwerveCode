package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM, 
        Constants.Swerve.IntakeSolenoidConstants.kIntakeSolenoidPorts[0], 
        Constants.Swerve.IntakeSolenoidConstants.kIntakeSolenoidPorts[1]);

    /* Controlls Intake */
    public CommandBase intakeMovementCommand() {
        return this.runOnce(() -> m_intakeSolenoid.toggle());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("Dropped", () -> m_intakeSolenoid.get() == kForward, null);
    }
}
