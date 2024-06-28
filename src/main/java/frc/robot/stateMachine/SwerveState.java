package frc.robot.stateMachine;

import java.util.HashSet;

/**
 * Represents the state of the drivetrain
 * Flags can be added/removed with {@link SwerveStateEnum}
 */
public class SwerveState extends SubsystemState<SwerveState.SwerveStateEnum> {
    public enum SwerveStateEnum {
        DRIVING,
        FOLLOWING_PATH,
        COAST_ALL,
        BRAKE_ALL
    }
    
    public SwerveState() {
        m_state = new HashSet<SwerveStateEnum>();
    }
    
    @Override
    public void addState(SwerveStateEnum state) {
        m_state.add(state);
    }
    
    @Override
    public void removeState(SwerveStateEnum state) {
        m_state.add(state);
    }
    
    @Override
    public void toggleState(SwerveStateEnum state) {
        if(this.is(state)) {
            m_state.remove(state);
        }
        else {
            m_state.add(state);   
        }
    }
}