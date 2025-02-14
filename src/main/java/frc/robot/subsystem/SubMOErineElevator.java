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

        this.extensionSensor = extensionSensor;
        this.setSensors(new ElevatorInputsAutoLogged());
        this.elevatorHeightMotor = elevatorHeightMotor;
        this.elevatorPivotMotor = elevatorPivotMotor;
        this.tiltEncoder = tiltEncoder;
        this.extensionSensor = extensionSensor;
    }

    @Override
    public void readSensors(ElevatorInputsAutoLogged sensors) {
        sensors.angle = Degree.of(elevatorPivotMotor.getAbsoluteEncoder().getPosition());
        sensors.horizontalSpeed = RPM.of(elevatorPivotMotor.getEncoder().getVelocity());
        sensors.extensionSpeed = InchesPerSecond.of(elevatorHeightMotor.getEncoder().getVelocity());
        sensors.elevatorVoltage =extensionSensor.getVoltage() / .3 ; //Todo: WHYYYYYYYY
        sensors.elevatorVoltageFromADC = String.format("%04x", extensionSensor.getValue());
        sensors.extension = getExtension();
        sensors.canGoDown = canGoDown();

    }

    @Override
    public void moveVertically(LinearVelocity speed) {
        elevatorHeightMotor.set(speed.in(InchesPerSecond));
    }

    @Override
    public Distance getExtension() {
        return Inch.of(extensionSensor.getVoltage());
    }



    public boolean canGoDown() {
        return  elevatorPivotMotor.getReverseLimitSwitch().isPressed();
    }

    @Override
    public Angle getAngle() {
        return tiltEncoder.getAbsolutePosition().getValue();
    }



    @Override
    public void moveHorizontally(AngularVelocity speed) {
        elevatorPivotMotor.set(speed.in(DegreesPerSecond));
    }
}
