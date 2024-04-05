package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.logging.LoggableDouble;
import frc.robot.util.logging.Logger;

public class WheelRadiusCharacterization extends Command {
    private final SwerveSubsystem m_swerve;
    private final LoggableDouble m_currentEffectiveWheelRadiusLog;
    private final DoubleSupplier m_gyroYawRadSupplier;
    private double m_lastGyroYawRad;
    private double m_accumGyroYawRad;

    private final SlewRateLimiter m_thetaLimiter;
    private double[] m_startingWheelPositionsRad;
    private double m_currentEffectiveWheelRadius;

    public WheelRadiusCharacterization(SwerveSubsystem swerve, DoubleSupplier gyroYawRadSupplier) {
        m_swerve = swerve;
        m_gyroYawRadSupplier = gyroYawRadSupplier;
        m_thetaLimiter = new SlewRateLimiter(2);
        m_currentEffectiveWheelRadiusLog = new LoggableDouble("Current Effective Wheel Radius", true);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        m_startingWheelPositionsRad = m_swerve.getWheelPoses();
        m_currentEffectiveWheelRadius = 0.0;
        Logger.addLoggable(m_currentEffectiveWheelRadiusLog);
        m_accumGyroYawRad = 0.0;
        m_lastGyroYawRad = m_gyroYawRadSupplier.getAsDouble();
        m_thetaLimiter.reset(0);
    }

    @Override
    public void execute() {
        m_swerve.drive(0,0, m_thetaLimiter.calculate(2*Math.PI), true);

        m_accumGyroYawRad += MathUtil.angleModulus(m_lastGyroYawRad - m_gyroYawRadSupplier.getAsDouble());
        m_lastGyroYawRad = m_gyroYawRadSupplier.getAsDouble();

        double[] wheelPositionsRad = m_swerve.getWheelPoses();
        double averageWheelPositionDelta = 0.0;
        for (int i = 0; i < 4; i++) {
            averageWheelPositionDelta += Math.abs(wheelPositionsRad[i] - m_startingWheelPositionsRad[i]);
          }
        averageWheelPositionDelta /= 4;

        m_currentEffectiveWheelRadius = (m_accumGyroYawRad * Constants.SwerveConstants.kDriveBaseRadius) / averageWheelPositionDelta;
        
        m_currentEffectiveWheelRadiusLog.log(m_currentEffectiveWheelRadius);

        System.out.println(
            "Effective Wheel Radius: "
                + m_currentEffectiveWheelRadius
                + " meters");

    }

    @Override
    public void end(boolean interrupted) {
        if (m_accumGyroYawRad <= Math.PI * 2.0) {
            System.out.println("Not enough data for characterization");
          } else {
            System.out.println(
                "Effective Wheel Radius: "
                    + m_currentEffectiveWheelRadius
                    + " meters");
          }
    }
}
