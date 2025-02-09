package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.*;

public class SubMOErineElevator extends MOESubsystem<ElevatorInputsAutoLogged> implements ElevatorControl {

    SparkMax elevatorHeightMotor;
    SparkMax elevatorPivotMotor;
    CANcoder tiltEncoder;

    AnalogInput extensionSensor;

    public SubMOErineElevator(
        SparkMax elevatorHeightMotor,
        SparkMax elevatorPivotMotor,
        CANcoder tiltEncoder,
        AnalogInput extensionSensor
    ) {
        this.setSensors(new ElevatorInputsAutoLogged());
        this.elevatorHeightMotor = elevatorHeightMotor;
        this.elevatorPivotMotor = elevatorPivotMotor;
        this.tiltEncoder = tiltEncoder;
        this.extensionSensor = extensionSensor;
    }

    @Override
    public void readSensors(ElevatorInputsAutoLogged sensors) {
        sensors.extension = getExtension();
        sensors.angle = tiltEncoder.getAbsolutePosition().getValue();
        sensors.height = getHeight();
        sensors.horizontalSpeed = RPM.of(elevatorPivotMotor.getEncoder().getVelocity());
        sensors.extensionSpeed = InchesPerSecond.of(elevatorHeightMotor.getEncoder().getVelocity());
        sensors.elevatorVoltage = Volts.of(extensionSensor.getVoltage());
        sensors.extension = getExtension();

    }

    @Override
    public void moveVertically(LinearVelocity speed) {
        elevatorHeightMotor.set(speed.in(InchesPerSecond));
    }

    @Override
    public Angle getAngle() {
        return tiltEncoder.getAbsolutePosition().getValue();
    }



    @Override
    public void moveHorizontally(AngularVelocity speed) {
        elevatorPivotMotor.set(speed.in(DegreesPerSecond));
    }

    @Override
    public Distance getPivotHeight() {
        return Inches.of(8);
    }

    public Distance getExtension() {
        return Feet.of(heightSensor.getVoltage());
        // 1 foot per 1 volt
    }
}
