package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.periodic.LoggedPIDController;

public record PIDConstants(double kp, double ki, double kd, double iZone, boolean enableContinuousInput, double minimumInput, double maximumInput, double positionTolerance, double velocityTolerance, double maximumIntegral, double minimumIntegral) {
    public PIDConstants(double kp, double ki, double kd, double iZone, boolean enableContinuousInput, double minimumInput, double maximumInput, double positionTolerance, double velocityTolerance) {
        this(kp, ki, kd, iZone, enableContinuousInput, minimumInput, maximumInput, positionTolerance, velocityTolerance, 1.0, -1.0);
    }

    public PIDConstants(double kp, double ki, double kd) {
        this(kp, ki, kd, Double.POSITIVE_INFINITY, false, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, 0.05, Double.POSITIVE_INFINITY);
    }

    /**
     * Constructs a {@link PIDController} object with the constants provided.
     * @param constants {@link PIDConstants} to make the {@link PIDController} out of
     * @return {@link PIDController} object with the constants of the given {@link PIDConstants}
     */
    public static PIDController getPIDController(PIDConstants constants) {
        PIDController controller = new PIDController(constants.kp(), constants.ki(), constants.kd());

        controller.setIZone(constants.iZone());
        if (constants.enableContinuousInput)
            controller.enableContinuousInput(constants.minimumInput(), constants.maximumInput());
        controller.setTolerance(constants.positionTolerance, constants.velocityTolerance);
        controller.setIntegratorRange(constants.minimumIntegral, constants.maximumIntegral);

        return controller;
    }

    /**
     * Constructs a {@link LoggedPIDController} object with the constants provided.
     * @param constants {@link PIDConstants} to make the {@link LoggedPIDController} out of.
     * @param name Name of the {@link LoggedPIDController} in NetworkTables
     * @return {@link LoggedPIDController} object with the constants provided.
     */
    public static LoggedPIDController getLoggedPIDController(PIDConstants constants, String name) {
        LoggedPIDController controller = new LoggedPIDController(name, constants.kp(), constants.ki(), constants.kd());

        controller.setIZone(constants.iZone());
        if (constants.enableContinuousInput)
            controller.enableContinuousInput(constants.minimumInput(), constants.maximumInput());
        controller.setTolerance(constants.positionTolerance, constants.velocityTolerance);
        controller.setIntegratorRange(constants.minimumIntegral, constants.maximumIntegral);

        return controller;
    }


    /**
     * Constructs a {@link PIDController} object with the constants of this object.
     * @param constants {@link PIDConstants} to make the {@link PIDController} out of
     * @return {@link PIDController} object with the constants of the given {@link PIDConstants}
     */
    public PIDController getPIDController() {
        return getPIDController(this);
    }

    /**
     * Constructs a {@link LoggedPIDController} object with the constants of this object.
     * @param name Name of the {@link LoggedPIDController} in NetworkTables
     * @return {@link LoggedPIDController} object with the constants of this object.
     */
    public LoggedPIDController getLoggedPIDController(String name) {
        return getLoggedPIDController(this, name);
    }
}
