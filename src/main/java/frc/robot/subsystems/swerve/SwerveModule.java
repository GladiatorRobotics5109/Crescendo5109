package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
   Translation2d getPos();
   void setDesiredState(SwerveModuleState state, boolean optimize); 
   void setDesiredState(SwerveModuleState state);
   void brakeAll();
   void coastAll();
   String getName();
   int getNumber();
   SwerveModuleState getState();
   default Rotation2d getAngle() {
    return getState().angle;
   }
   SwerveModulePosition getModulePosition();
}
