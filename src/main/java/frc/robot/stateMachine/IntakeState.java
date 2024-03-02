package frc.robot.stateMachine;

import java.util.HashSet;

public class IntakeState extends SubsystemState<IntakeState.IntakeStateEnum> {
    public enum IntakeStateEnum {
        INTAKING,
        REVERSING
    }
    
    public IntakeState() {
        m_state = new HashSet<IntakeStateEnum>();
    }
    
    @Override
    public void addState(IntakeStateEnum state) {
        m_state.add(state);
    }
    
    @Override
    public void removeState(IntakeStateEnum state) {
        m_state.remove(state);
    }
    
    @Override
    public void toggleState(IntakeStateEnum state) {
        if(this.is(state)) {
            m_state.remove(state);
        }
        else {
            m_state.add(state);
        }
    }
}