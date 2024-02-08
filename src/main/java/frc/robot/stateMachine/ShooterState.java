package frc.robot.stateMachine;


import java.util.HashSet;
import java.util.Set;

public class ShooterState {
    public enum ShooterStateEnum {
        RESTING,
        SHOOTER_WHEEL_SPINNING,
        FEEDER_WHEELS_SPINING,
        HAS_NOTE,
        AUTO_AIMING;
    }
    
    private final Set<ShooterStateEnum> m_state;
    
    public ShooterState() {
        m_state = new HashSet<ShooterStateEnum>();
    }
    
    public void addState(ShooterStateEnum state) {
        m_state.add(state);
    }
    
    public void removeState(ShooterStateEnum state) {
        m_state.remove(state);
    }
    
    public void toggleState(ShooterStateEnum state) {
        if(this.is(state)) {
            m_state.remove(state);
        }
        else {
            m_state.add(state);
        }
    }
    
    public boolean is(ShooterStateEnum state) {
        return m_state.contains(state);
    }
    
    public boolean is(ShooterStateEnum... states) {
        for(ShooterStateEnum state : states) {
            if (!this.is(state))
                return false;
        }
        
        return true;
    }
}