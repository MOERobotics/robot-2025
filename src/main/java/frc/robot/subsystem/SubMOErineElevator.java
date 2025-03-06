package frc.robot.subsystem;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.ElevatorControl;
import frc.robot.subsystem.interfaces.ElevatorInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class SubMOErineElevator extends MOESubsystem<ElevatorInputsAutoLogged> implements ElevatorControl {

    public SparkMax elevatorExtensionMotor;
    public SparkMax elevatorPivotMotor;
    public SparkAbsoluteEncoder tiltEncoder;
    public AnalogInput extensionSensor;

    public AddressableLED addressableLED;
    public AddressableLEDBuffer addressableLEDBuffer;


    public SubMOErineElevator(
        SparkMax elevatorExtensionMotor,
        SparkMax elevatorPivotMotor,
        AnalogInput extensionSensor
    ) {
        super(new ElevatorInputsAutoLogged());
        this.elevatorExtensionMotor = elevatorExtensionMotor;
        this.elevatorPivotMotor = elevatorPivotMotor;
        SparkMaxConfig extensionMotorConfig = new SparkMaxConfig();
        SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();

        this.tiltEncoder = elevatorPivotMotor.getAbsoluteEncoder();
        this.extensionSensor = extensionSensor;

        extensionMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);
        extensionMotorConfig.limitSwitch.reverseLimitSwitchEnabled(false);
        pivotMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);

        elevatorPivotMotor.configure(pivotMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        elevatorExtensionMotor.configure(extensionMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        addressableLED = new AddressableLED(7);// TODO CHANGE PORT
        addressableLED.setBitTiming(320, 700, 700, 320);
        addressableLEDBuffer = new AddressableLEDBuffer(300);
        addressableLED.setLength(addressableLEDBuffer.getLength());
        addressableLED.setData(addressableLEDBuffer);
        addressableLED.start();

    }

    @Override
    public void readSensors(ElevatorInputsAutoLogged sensors) {
        sensors.height = getHeight();
        sensors.angle = Degrees.of(elevatorPivotMotor.getAbsoluteEncoder().getPosition() * -0.173 + 5.78).minus(Radians.of(0.1));
        sensors.horizontalSpeed = RPM.of(elevatorPivotMotor.getEncoder().getVelocity());
        sensors.extensionSpeed = InchesPerSecond.of(elevatorExtensionMotor.getEncoder().getVelocity());
        sensors.elevatorVoltage = Volts.of(extensionSensor.getVoltage());
        sensors.elevatorVoltageFromADC = String.format("%04x", extensionSensor.getValue());
        sensors.extension = Centimeters.of((getSensors().elevatorVoltage.in(Volts) * 36.46529) + 27.36326);
        sensors.canGoDown = elevatorExtensionMotor.getReverseLimitSwitch().isPressed();
        sensors.canGoUp = sensors.elevatorVoltage.lt(Volts.of(4.248));
        sensors.canGoRight = sensors.angle.gt(Degrees.of(9.55));
        sensors.canGoLeft = sensors.angle.lt(Degrees.of(52.42));
        sensors.extensionMotorPosition = Rotations.of(elevatorExtensionMotor.getEncoder().getPosition());
        Logger.recordOutput("ElevatorAngleDegrees", sensors.angle.in(Degrees));

        SmartDashboard.putBoolean("ElevatorCanGoUp",canGoUP());
        SmartDashboard.putBoolean("ElevatorCanGoDown",canGoDown());

        SmartDashboard.putNumber("Elevator Extension", sensors.extension.in(Centimeters));
        SmartDashboard.putNumber("Elevator Angle", sensors.angle.in(Degrees));
    }

    @Override
    public void moveVertically(LinearVelocity speed) {
        if (
            ( canGoUP()   && speed.gt(FeetPerSecond.zero()) ) ||
            ( canGoDown() && speed.lt(FeetPerSecond.zero()) )
        ) {
            elevatorExtensionMotor.set(speed.in(FeetPerSecond));
        } else {
            elevatorExtensionMotor.set(0);
        }
    }

    @Override
    public void moveHorizontally(AngularVelocity speed) {
        if (
            ( canGoRight() && speed.gt(DegreesPerSecond.zero()) ) ||
            ( canGoLeft()  && speed.lt(DegreesPerSecond.zero()) )
        ) {
            elevatorPivotMotor.set(speed.in(DegreesPerSecond));
        } else {
            elevatorPivotMotor.set(0);
        }
    }

    @Override
    public Distance getPivotHeight() {
        return Centimeters.of(8);
    }

    @Override
    public void setLEDColor(Color color) {
        LEDPattern LEDPatternColor = LEDPattern.solid(color);
        LEDPatternColor.applyTo(addressableLEDBuffer);
        addressableLED.setData(addressableLEDBuffer);
        Logger.recordOutput("LED Color", color.toHexString());

    }

    @Override
    public void periodic() {
        super.periodic();
        setLEDColor(Color.kGreen);
    }
}
