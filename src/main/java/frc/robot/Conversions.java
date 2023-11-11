package frc.robot;

public class Conversions {
    public static double degToKraken(double deg, double gearRatio) {
        return deg / (360.0 / (gearRatio * 2000.0));
    }
    public static double krakenToRad(double kraken, double gearRatio) {
        // kraken * (1 / tik/rad)
        return gearRatio * kraken / Constants.SwerveConstants.kKrakenTicksPerMotorRadian;
    }
}
