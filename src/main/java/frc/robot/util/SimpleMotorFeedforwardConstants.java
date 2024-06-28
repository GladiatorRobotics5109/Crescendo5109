package frc.robot.util;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public record SimpleMotorFeedforwardConstants(double ks, double kv, double ka) {
    /**
     *
     * @param constants {@link SimpleMotorFeedforwardConstants} to make the {@link SimpleMotorFeedforward} out of
     * @return {@link SimpleMotorFeedforward} object with the constants of the given {@link SimpleMotorFeedforwardConstants}
     */
    public static SimpleMotorFeedforward get(SimpleMotorFeedforwardConstants constants) {
        return new SimpleMotorFeedforward(
            constants.ks(),
            constants.kv(),
            constants.ka()
        );
    }

    /**
     * @return {@link SimpleMotorFeedforward} object with the constants of this object
     */
    public SimpleMotorFeedforward get() {
        return get(this);
    }
}
