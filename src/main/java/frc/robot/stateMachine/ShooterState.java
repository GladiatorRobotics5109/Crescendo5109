package frc.robot.stateMachine;


import java.util.HashSet;

public class ShooterState extends SubsystemState<ShooterState.ShooterStateEnum> {
    public enum ShooterStateEnum {
        SHOOTER_WHEEL_SPINNING,
        FEEDER_WHEELS_SPINNING,
        REVERSE_BOTH,
        HAS_NOTE,
        AUTO_AIMING;
    }
    
    public ShooterState() {
        m_state = new HashSet<ShooterStateEnum>();
    }
    
    @Override
    public void addState(ShooterStateEnum state) {
        m_state.add(state);
    }
    
    @Override
    public void removeState(ShooterStateEnum state) {
        m_state.remove(state);
    }
    
    @Override
    public void toggleState(ShooterStateEnum state) {
        if(this.is(state)) {
            m_state.remove(state);
        }
        else {
            m_state.add(state);
        }
    }
}