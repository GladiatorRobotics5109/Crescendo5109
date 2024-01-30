package frc.robot;

public class Conversions {
    /**
     * Converts radians to kraken ticks with a given gear ratio
    */
    public static double radToKraken(double rad, double gearRatio) {
        return gearRatio * (rad * Constants.ModuleConstants.kKrakenTicksPerMotorRadian);
    }

    /**
     * Converts kraken ticks to radians with a given gear ratio
     * @param kraken
     * @param gearRatio
     * @return
     */
    public static double krakenToRad(double kraken, double gearRatio) {
        return gearRatio * (kraken / Constants.ModuleConstants.kKrakenTicksPerMotorRadian);
    }

    public static double wheelToMeters(double wheel) {
        return wheel * (Constants.ModuleConstants.kWheelRadius * 2 * Math.PI);
    }
}
