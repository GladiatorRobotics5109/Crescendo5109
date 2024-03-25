package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Constants;

public class WheelRadiusCharacterization extends Command {
    private final SwerveSubsystem m_swerve;
    private final DoubleSupplier m_gyroYawRadSupplier;
    private double m_lastGyroYawRad;
    private double m_accumGyroYawRad;

    private final SlewRateLimiter m_thetaLimiter;
    private double[] m_startingWheelPositionsRad;
    private double currentEffectiveWheelRadius;

    public WheelRadiusCharacterization(SwerveSubsystem swerve, DoubleSupplier gyroYawRadSupplier) {
        m_swerve = swerve;
        m_gyroYawRadSupplier = gyroYawRadSupplier;
        m_thetaLimiter = new SlewRateLimiter(1.0);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        m_startingWheelPositionsRad = m_swerve.getWheelPoses();
        currentEffectiveWheelRadius = 0.0;
        m_accumGyroYawRad = 0.0;
        m_lastGyroYawRad = m_gyroYawRadSupplier.getAsDouble();
        m_thetaLimiter.reset(0);
    }

    @Override
    public void execute() {
        m_swerve.drive(0,0, m_thetaLimiter.calculate(0.1), true);

        m_accumGyroYawRad += MathUtil.angleModulus(m_lastGyroYawRad - m_gyroYawRadSupplier.getAsDouble());
        m_lastGyroYawRad = m_gyroYawRadSupplier.getAsDouble();

        double[] wheelPositionsRad = m_swerve.getWheelPoses();
        double averageWheelPositionDelta = 0.0;
        for (int i = 0; i < 4; i++) {
            averageWheelPositionDelta += Math.abs(wheelPositionsRad[i] - m_startingWheelPositionsRad[i]);
          }
        averageWheelPositionDelta /= 4;

        currentEffectiveWheelRadius = (m_accumGyroYawRad * Constants.SwerveConstants.kDriveBaseRadius) / averageWheelPositionDelta;
        
    }

    @Override
    public void end(boolean interrupted) {
        if (m_accumGyroYawRad <= Math.PI * 2.0) {
            System.out.println("Not enough data for characterization");
          } else {
            System.out.println(
                "Effective Wheel Radius: "
                    + currentEffectiveWheelRadius
                    + " meters");
          }
    }
}
