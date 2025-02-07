package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.*;

public class SubMOErineElevator extends MOESubsystem<ElevatorInputsAutoLogged> implements ElevatorControl {

    SparkMax elevatorHeightMotor;
    SparkMax elevatorPivotMotor;
    CANcoder tiltEncoder;
    AnalogInput heightSensor;

    public SubMOErineElevator(
        SparkMax elevatorHeightMotor,
        SparkMax elevatorPivotMotor,
        CANcoder tiltEncoder
    ) {
        this.setSensors(new ElevatorInputsAutoLogged());
        this.elevatorHeightMotor = elevatorHeightMotor;
        this.elevatorPivotMotor = elevatorPivotMotor;
        this.tiltEncoder = tiltEncoder;
        this.heightSensor = new AnalogInput(1);
    }

    @Override
    public void readSensors(ElevatorInputsAutoLogged sensors) {
        sensors.extension = _getExtension();
        sensors.angle = tiltEncoder.getAbsolutePosition().getValue();
        sensors.height = getPivotHeight();
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

    @Override
    public Distance getPivotHeight() {
        return Inches.of(8);
    }

    public Distance _getExtension() {
        return Feet.of(heightSensor.getVoltage());
        // 1 foot per 1 volt
    }
}
