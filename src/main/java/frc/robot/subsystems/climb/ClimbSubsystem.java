package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.stateMachine.ClimbState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.util.Constants;
import frc.robot.util.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    private final CANSparkMax m_leftMotor;
    private final CANSparkMax m_rightMotor;

    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;

    private final SparkPIDController m_leftPIDController;
    private final SparkPIDController m_rightPIDController;

    private final ClimbState m_state;

    public ClimbSubsystem() {

        m_state = StateMachine.getClimbState();

        m_leftMotor = new CANSparkMax(ClimbConstants.kLeftClimbMotorPort, MotorType.kBrushless);
        m_rightMotor = new CANSparkMax(ClimbConstants.kRightClimbMotorPort, MotorType.kBrushless);

        m_leftEncoder = m_leftMotor.getEncoder();
        m_rightEncoder = m_rightMotor.getEncoder();

        m_leftPIDController = m_leftMotor.getPIDController();
        m_rightPIDController = m_rightMotor.getPIDController();

        m_leftPIDController.setP(ClimbConstants.kP);
        m_leftPIDController.setP(ClimbConstants.kI);
        m_leftPIDController.setP(ClimbConstants.kD);

        m_rightPIDController.setP(ClimbConstants.kP);
        m_rightPIDController.setP(ClimbConstants.kI);
        m_rightPIDController.setP(ClimbConstants.kD);


        m_leftEncoder.setPositionConversionFactor(ClimbConstants.kClimbPositionConversionFactor);
        m_rightEncoder.setPositionConversionFactor(ClimbConstants.kClimbPositionConversionFactor);

        m_leftEncoder.setPosition(ClimbConstants.kMaxExtension);
        m_rightEncoder.setPosition(ClimbConstants.kMaxExtension);
    }

    public Command getRetractCommand() {
        return this.runOnce(()->{
            setExtension(ClimbConstants.kMinExtension);
        });
    }

    public Command getExtendCommand() {
        return this.runOnce(()->{
            setExtension(ClimbConstants.kMaxExtension);
        });
    }

    public Command getSetExtensionCommand(double extension) {
        return this.runOnce(()->{
            setExtension(extension);
        });
    }

    public Command getIncreaseLeftExtensionCommand() {
        return this.runOnce(() -> increaseLeftExtension());
    }

    public Command getDecreaseLeftExtensionCommand() {
        return this.runOnce(() -> decreaseLeftExtension());
    }


    public Command getIncreaseRightExtensionCommand() {
        return this.runOnce(() -> increaseRightExtension());
    }

    public Command getDecreaseRightExtensionCommand() {
        return this.runOnce(() -> decreaseRightExtension());
    }

    public Command getIncreaseExtensionCommand() {
        return this.runOnce(() -> {
            increaseLeftExtension();
            increaseRightExtension();
        });
    }

    public Command getDecreaseExtensionCommand() {
        return this.runOnce(() -> {
            decreaseLeftExtension();
            decreaseRightExtension();
        });
    }

    public void setLeftExtension(double extension) {
        if (extension <= ClimbConstants.kMaxExtension && extension >= ClimbConstants.kMinExtension) {
            m_leftPIDController.setReference(extension, ControlType.kPosition);
        }
    }

    public void setRightExtension(double extension) {
        if (extension <= ClimbConstants.kMaxExtension && extension >= ClimbConstants.kMinExtension) {
            m_rightPIDController.setReference(extension, ControlType.kPosition);
        }
    }

    public void increaseLeftExtension() {
        setLeftExtension(m_leftEncoder.getPosition() + Units.inchesToMeters(1));
    }

    public void decreaseLeftExtension() {
        setLeftExtension(m_leftEncoder.getPosition() - Units.inchesToMeters(1));
    }

    public void increaseRightExtension() {
        setRightExtension(m_rightEncoder.getPosition() + Units.inchesToMeters(1));
    }

    public void decreaseRightExtension() {
        setRightExtension(m_rightEncoder.getPosition() - Units.inchesToMeters(1));
    }

    public void setExtension(double extension) {
        setLeftExtension(extension);
        setRightExtension(extension);
    }

}
