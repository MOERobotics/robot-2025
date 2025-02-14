package frc.robot.utils;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import lombok.Builder;

@Builder
public record FeedforwardConstants(double kS, double kV, double kA, double kG) {
    public FeedforwardConstants(double kS, double kV, double kA) {
        this(kS, kV, kA, 0.0);
    }

    public SimpleMotorFeedforward makeSimpleFeedforward() {
        return new SimpleMotorFeedforward(this.kS, this.kV, this.kA);
    }

    public ArmFeedforward makeArmFeedforward() {
        return new ArmFeedforward(this.kS, this.kG, this.kV, this.kA);
    }

    public ElevatorFeedforward makeElevatorFeedforward() {
        return new ElevatorFeedforward(this.kS, this.kG, this.kV, this.kA);
    }
}
