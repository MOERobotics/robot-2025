package frc.robot.subsystem;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.*;

public class CoralCollector extends MOESubsystem<CoralCollectorInputsAutoLogged> implements CoralCollectorControl {
    SparkMax coralHeadLeft, coralHeadRight;

    public CoralCollector(SparkMax coralHeadLeft, SparkMax coralHeadRight) {
        this.setSensors(new CoralCollectorInputsAutoLogged());
        this.coralHeadLeft = coralHeadLeft;
        this.coralHeadRight = coralHeadRight;
        getSensors().hasCoral = false;
        getSensors().inFrontReef = false;
        getSensors().velocityRight = RPM.zero();
        getSensors().velocityLeft = RPM.zero();
    }

    @Override
    public void readSensors(CoralCollectorInputsAutoLogged sensors) {
        sensors.hasCoral = coralHeadRight.getReverseLimitSwitch().isPressed();
        sensors.inFrontReef = false;
        sensors.velocityLeft = RPM.of(coralHeadLeft.getEncoder().getVelocity());
        sensors.velocityRight = RPM.of(coralHeadRight.getEncoder().getVelocity());
    }

    @Override
    public void setCoralVelocity(AngularVelocity leftPower, AngularVelocity rightPower) {
        coralHeadLeft.set(leftPower.in(RPM));
        coralHeadLeft.set(rightPower.in(RPM));
    }
}
