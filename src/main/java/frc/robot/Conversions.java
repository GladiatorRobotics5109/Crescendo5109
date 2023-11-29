package frc.robot;

public class Conversions {
    /**
     * Converts radians to kraken ticks with a given gear ratio
     * @param rad radians
     * @param gearRatio gear ratio to apply
     * @return result in kraken ticks
    */
    public static double radToKraken(double rad, double gearRatio) {
        return gearRatio * (rad * Constants.SwerveConstants.kKrakenTicksPerMotorRadian);
    }

    /**
     * Converts kraken ticks to radians with a given gear ratio
     * @param kraken kraken ticks
     * @param gearRatio gear ratio to apply
     * @return result in radians
     */
    public static double krakenToRad(double kraken, double gearRatio) {
        return gearRatio * (kraken / Constants.SwerveConstants.kKrakenTicksPerMotorRadian);
    }
}
