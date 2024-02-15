package frc.robot.stateMachine;

import java.util.HashSet;
import java.util.Set;

public class SwerveState {
    public enum SwerveStateEnum {
        DRIVING,
        FOLLOWING_PATH,
        COAST_ALL,
        BRAKE_ALL
    }
    
    private final Set<SwerveStateEnum> m_state;
    
    public SwerveState() {
        m_state = new HashSet<SwerveStateEnum>();
    }
    
    public void addState(SwerveStateEnum state) {
        m_state.add(state);
    }
    
    public void removeState(SwerveStateEnum state) {
        m_state.add(state);
    }
    
    public void toggleState(SwerveStateEnum state) {
        if(this.is(state)) {
            m_state.remove(state);
        }
        else {
            m_state.add(state);   
        }
    }
    
    public boolean is(SwerveStateEnum state) {
        return m_state.contains(state);
    }
    
    public boolean is(SwerveStateEnum... states) {
        for(SwerveStateEnum state : states) {
            if(!this.is(state))
                return false;
        }
        
        return true;
    }
}