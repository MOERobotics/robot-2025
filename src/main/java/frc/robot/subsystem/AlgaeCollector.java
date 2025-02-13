package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.*;

public class AlgaeCollector extends MOESubsystem<AlgaeCollectorInputsAutoLogged> implements AlgaeCollectorControl {
    ;
    SparkMax algaeWheel, algaeArm;
    CANcoder algaeArmEncoder;
    WheelState wheelState;
    Angle startAngle, collectAngle, tolerance;

    public AlgaeCollector(SparkMax algaeWheel, SparkMax algaeArm, CANcoder algaeArmEncoder, Angle startAngle, Angle collectAngle, Angle tolerance) {
        this.setSensors(new AlgaeCollectorInputsAutoLogged());
        this.algaeWheel = algaeWheel;
        this.algaeArm = algaeArm;
        this.algaeArmEncoder = algaeArmEncoder;
        this.startAngle = startAngle;
        this.collectAngle = collectAngle;
        this.tolerance = tolerance;
    }

    @Override
    public void readSensors(AlgaeCollectorInputsAutoLogged sensors) {
        sensors.wheelAppliedVolts = Volts.of(algaeWheel.getAppliedOutput() * algaeArm.getBusVoltage());
        sensors.wheelVelocity = RPM.of(algaeWheel.getEncoder().getVelocity());
        sensors.algaeArmAppliedVolts = Volts.of(algaeArm.getAppliedOutput() * algaeArm.getBusVoltage());
        sensors.algaeArmAngle = algaeArmEncoder.getAbsolutePosition().getValue();
        sensors.hasAlgae = algaeWheel.getReverseLimitSwitch().isPressed();
        sensors.inStartPosition = getArmAngle().isNear(startAngle, tolerance);
        sensors.inCollectPosition = getArmAngle().isNear(collectAngle, tolerance);
        sensors.wheelState = wheelState;
    }

    @Override
    public void setArmVelocity(AngularVelocity armVelocity) {
        algaeArm.set(armVelocity.in(RPM));
    }


    @Override
    public void setWheelVelocity(AngularVelocity wheelVelocity) {
        algaeWheel.set(wheelVelocity.in(RPM));
        if (wheelVelocity.gt(RPM.zero())) {
            setWheelState(WheelState.COLLECTING);
        } else if (wheelVelocity.lt()t(RPM.zero())) {
            setWheelState(WheelState.EJECTING);
        } else {
            setWheelState(WheelState.HOLDING);
        }
    }

    @Override
    public void setWheelState(WheelState wheelState) {
        this.wheelState = wheelState;
    }
}
