package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    public CANSparkMax m_intakeMotor;
    
    public IntakeSubsystem(int intakeMotorPort) {
        m_intakeMotor = new CANSparkMax(intakeMotorPort, CANSparkLowLevel.MotorType.kBrushless);
    }
    
    public void startIntake() {
        m_intakeMotor.set(0.5);
    }
    
    public void stopIntake() {
        m_intakeMotor.set(0.0);
    }
    
    public Command getStartIntakeCommand() {
        return this.runOnce(() -> startIntake()).withName("startIntakeCommand");
    }
    
    public Command getStopIntakeCommand() {
        return this.runOnce(() -> stopIntake()).withName("stopIntakeCommand");
    }
}