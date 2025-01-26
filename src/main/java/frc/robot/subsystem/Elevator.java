package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.*;

public class Elevator extends MOESubsystem<ElevatorInputsAutoLogged> implements ElevatorSubsystem {

SparkMax height;
SparkMax pivot;
CANcoder tilt;

    public Elevator(
            SparkMax height,
            SparkMax pivot,
            CANcoder tilt
    ){
        this.setSensors(new ElevatorInputsAutoLogged());
this.height = height;
this.pivot = pivot;
this.tilt = tilt;
}

    @Override
    public void readSensors(ElevatorInputsAutoLogged sensors) {
        sensors.angle = tilt.getAbsolutePosition().getValue();
        sensors.horizontalSpeed = RPM.of(pivot.getEncoder().getVelocity());
        sensors.verticalSpeed = InchesPerSecond.of(height.getEncoder().getVelocity());
    }

    @Override
    public void moveVertically(LinearVelocity speed) {
height.set(speed.in(InchesPerSecond));
    }

    @Override
    public void moveHorizontally(AngularVelocity speed) {
pivot.set(speed.in(DegreesPerSecond));
    }
}
