package frc.robot.subsystem;

import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.RPM;

public class SimulationHelpers {
    public static AngularVelocity decelerate(AngularVelocity velocity, double decelerationCoef) {
        if (velocity.isNear(RPM.zero(), 0.01)) {
            return RPM.zero();
        } else {
            return velocity.abs(RPM) > decelerationCoef ? velocity.minus(RPM.of(Math.copySign(decelerationCoef, velocity.in(RPM)))) : RPM.zero();
        }
    }

    public static AngularVelocity getSystemVelocity(AngularVelocity motorVelocity, double gearing){
        return motorVelocity.div(gearing);
    }

    public static AngularVelocity getMotorVelocity(AngularVelocity systemVelocity, double gearing){
        return systemVelocity.times(gearing);
    }
}
