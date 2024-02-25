package frc.robot.stateMachine;

import java.util.Set;

/**
 * Abstract class that represents the state of a {@link edu.wpi.first.wpilibj2.command.SubsystemBase}
 */
public abstract class SubsystemState<T extends Enum<?>> {
    protected Set<T> m_state;
    
    public abstract void addState(T state);
    
    public abstract void removeState(T state);
    
    public abstract void toggleState(T state);
    
    /**
     * @return true if the subsystem is in the given state
     */
    public boolean is(T state) {
        return m_state.contains(state);
    }
    
    // /**
    //  * @apiNote This function asserts {@link SafeVarargs} because its users will probably not cause a heap pollution and throw a {@link ClassCastException}
    //  */
    // @SafeVarargs
    // public final boolean is(T... states) {
    //     for(T state : states) {
    //         if(!this.is(state))
    //             return false;
    //     }
        
    //     return true;
    // }
}