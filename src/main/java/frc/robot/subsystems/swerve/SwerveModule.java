package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModule {
    public abstract Translation2d getPos();
    
    public abstract void setDesiredState(SwerveModuleState state);
    
    public abstract void brakeAll();
   
    public abstract void coastAll();

    public abstract String getName();

    public abstract int getNumber();
}
