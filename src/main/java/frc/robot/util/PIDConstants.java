package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;

public record PIDConstants(double kp, double ki, double kd) {
    /**
     *
     * @param constants {@link PIDConstants} to make the {@link PIDController} out of
     * @return {@link PIDController} object with the constants of the given {@link PIDConstants}
     */
    public static PIDController get(PIDConstants constants) {
        return new PIDController(
            constants.kp(),
            constants.ki(),
            constants.kd()
        );
    }


    /**
     *
     * @return {@link PIDController} object with the constants of this object
     */
    public PIDController get() {
        return get(this);
    }
}
