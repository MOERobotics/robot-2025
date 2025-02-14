package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import lombok.Builder;

@Builder
public record PIDConstants(double kP, double kI, double kD, double kIMax) {
    public PIDController makeController() {
        return new PIDController(this.kP, this.kI, this.kD);
    }

    public PIDConstants(double kP, double kI, double kD) {
        this(kP, kI, kD, -1);
    }
}

