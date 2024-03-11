package frc.robot.stateMachine;

import java.util.HashSet;

public class ClimbState extends SubsystemState<ClimbState.ClimbStateEnum> {
    public enum ClimbStateEnum {
        INTAKING,
        REVERSING
    }
    
    public ClimbState() {
        m_state = new HashSet<ClimbStateEnum>();
    }
    
    @Override
    public void addState(ClimbStateEnum state) {
        m_state.add(state);
    }
    
    @Override
    public void removeState(ClimbStateEnum state) {
        m_state.remove(state);
    }
    
    @Override
    public void toggleState(ClimbStateEnum state) {
        if(this.is(state)) {
            m_state.remove(state);
        }
        else {
            m_state.add(state);
        }
    }
}