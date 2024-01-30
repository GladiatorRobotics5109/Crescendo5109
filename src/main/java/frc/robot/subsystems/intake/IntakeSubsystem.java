package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.AccelStrategy;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax m_intakeMotor;

    private final RelativeEncoder m_encoder;

    private final SparkPIDController m_pidController;
    
    public IntakeSubsystem(int intakeMotorPort) {
        m_intakeMotor = new CANSparkMax(intakeMotorPort, MotorType.kBrushless);

        m_intakeMotor.setIdleMode(IdleMode.kCoast);

        m_pidController = m_intakeMotor.getPIDController();

        m_pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    }
    
    public void startIntake() {
        m_pidController.setReference(1, ControlType.kSmartVelocity);
    }
    
    public void stopIntake() {
        m_pidController.setReference(0, ControlType.kSmartVelocity);
    }
    
    public Command getStartIntakeCommand() {
        return this.runOnce(() -> startIntake()).withName("startIntakeCommand");
    }
    
    public Command getStopIntakeCommand() {
        return this.runOnce(() -> stopIntake()).withName("stopIntakeCommand");
    }
}