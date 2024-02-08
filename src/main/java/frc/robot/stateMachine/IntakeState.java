package frc.robot.stateMachine;

import java.util.HashSet;
import java.util.Set;

public class IntakeState {
    public enum IntakeStateEnum {
        RESTING,
        INTAKING
    }
    
    private final Set<IntakeStateEnum> m_state;
    
    public IntakeState() {
        m_state = new HashSet<IntakeStateEnum>();
    }
    
    public void addState(IntakeStateEnum state) {
        m_state.add(state);
    }
    
    public void removeState(IntakeStateEnum state) {
        m_state.remove(state);
    }
    
    public void toggleState(IntakeStateEnum state) {
        if(this.is(state)) {
            m_state.remove(state);
        }
        else {
            m_state.add(state);
        }
    }
    
    public boolean is(IntakeStateEnum state) {
        return m_state.contains(state);
    }
    
    public boolean is(IntakeStateEnum... states) {
        for (IntakeStateEnum state : states) {
            if (!this.is(state))
                return false;
        }
        
        return true;
    }
}