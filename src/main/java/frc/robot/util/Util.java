package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class Util {
    /**
     * Guarantees that Alliance will be a valid value by returning
     * Constants.kDefaultAlliance if DriverStation.getAlliance() is empty
     *
     * @return an {@link Alliance} object instead of the optional returned by
     *         DriverStation.getAlliance()
     */
    public static Alliance getAllianceGuaranteed() {
        return DriverStation.getAlliance().orElse(Constants.kDefaultAlliance);
    }
}
