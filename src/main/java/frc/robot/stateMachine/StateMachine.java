package frc.robot.stateMachine;

/**
 * Represents the entire robot state
 */
public final class StateMachine {
    private static SwerveState s_swerveState;
    private static ShooterState s_shooterState;
    private static IntakeState s_intakeState;
    private static ClimbState s_climbState;

    public static void init() {
        s_swerveState = new SwerveState();
        s_shooterState = new ShooterState();
        s_intakeState = new IntakeState();
        s_climbState = new ClimbState();
    }
    
    public static SwerveState getSwerveState() {
        return s_swerveState;
    }
    
    public static ShooterState getShooterState() {
        return s_shooterState;
    }
    
    public static IntakeState getIntakeState() {
        return s_intakeState;
    }

    public static ClimbState getClimbState() {
        return s_climbState;
    }
 }