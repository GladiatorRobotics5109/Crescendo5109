package frc.robot;

import javax.sql.rowset.spi.TransactionalWriter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {


    private final Translation2d m_modulePos;
    private final String m_moduleName;
    private final int m_moduleNum;


    public SwerveModule(Translation2d modulePos, String moduleName, int moduleNum) {
        m_modulePos = modulePos;
        m_moduleName = moduleName;
        m_moduleNum = moduleNum;

    }

    public Translation2d getPos() {
        return m_modulePos;
    }

    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState optimizedState = RevOptomizer.optimize(state, new Rotation2d(m_turningEncoderRelative.getPosition()))
    }
}
