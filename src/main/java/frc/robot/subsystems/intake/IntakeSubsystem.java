package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.IntakeConstants;
import frc.robot.stateMachine.IntakeState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.stateMachine.IntakeState.IntakeStateEnum;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax m_intakeMotor;

    private final SparkPIDController m_pidController;    

    private final IntakeState m_state;
    
    
    public IntakeSubsystem() {

        m_state = StateMachine.getIntakeState();

        m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);

        m_intakeMotor.setIdleMode(IdleMode.kCoast);

        m_pidController = m_intakeMotor.getPIDController();

        m_pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        
    }
    
    public void startIntake() {
        m_pidController.setReference(1, ControlType.kSmartVelocity);
        m_state.addState(IntakeStateEnum.INTAKING);
    }
    
    public void stopIntake() {
        m_pidController.setReference(0, ControlType.kSmartVelocity);
        m_state.removeState(IntakeStateEnum.INTAKING);
    }
    
    public Command getStartIntakeCommand() {
        return this.runOnce(() -> startIntake()).withName("startIntakeCommand");
    }
    
    public Command getStopIntakeCommand() {
        return this.runOnce(() -> stopIntake()).withName("stopIntakeCommand");
    }
}