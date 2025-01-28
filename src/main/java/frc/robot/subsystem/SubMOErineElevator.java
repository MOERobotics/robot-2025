package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.*;

public class SubMOErineElevator extends MOESubsystem<ElevatorInputsAutoLogged> implements ElevatorControl {

SparkMax elevatorHeightMotor;
SparkMax elevatorPivotMotor;
CANcoder tiltEncoder;

    public SubMOErineElevator(
            SparkMax elevatorHeightMotor,
            SparkMax elevatorPivotMotor,
            CANcoder tiltEncoder
    ){
        this.setSensors(new ElevatorInputsAutoLogged());
this.elevatorHeightMotor = elevatorHeightMotor;
this.elevatorPivotMotor = elevatorPivotMotor;
this.tiltEncoder = tiltEncoder;
}

    @Override
    public void readSensors(ElevatorInputsAutoLogged sensors) {
        sensors.angle = tiltEncoder.getAbsolutePosition().getValue();
        sensors.horizontalSpeed = RPM.of(elevatorPivotMotor.getEncoder().getVelocity());
        sensors.extensionSpeed = InchesPerSecond.of(elevatorHeightMotor.getEncoder().getVelocity());
    }

    @Override
    public void moveVertically(LinearVelocity speed) {
elevatorHeightMotor.set(speed.in(InchesPerSecond));
    }

    @Override
    public void moveHorizontally(AngularVelocity speed) {
elevatorPivotMotor.set(speed.in(DegreesPerSecond));
    }
}
