package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModule {
    /** @return relative position of module from robot center */
    public abstract Translation2d getPoseRelative();
    
    public abstract void setDesiredState(SwerveModuleState state);

    /** @return absolute module position */
    public abstract SwerveModulePosition getModulePose();
    
    public abstract void brakeAll();
   
    public abstract void coastAll();

    public abstract String getName();

    public abstract int getNumber();

    /**
    * Reset turn encoders if it is relative
    */
    public abstract void resetTurnEncoder();

    public abstract SwerveModuleState getState();
}
