package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.ElevatorControl;
import frc.robot.subsystem.interfaces.ElevatorInputsAutoLogged;

import static edu.wpi.first.units.Units.*;

public class SubMOErineElevator extends MOESubsystem<ElevatorInputsAutoLogged> implements ElevatorControl {

    public SparkMax elevatorExtensionMotor;
    public SparkMax elevatorPivotMotor;
    public SparkAbsoluteEncoder tiltEncoder;

    public AnalogInput extensionSensor;

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
    }

    @Override
    public void readSensors(ElevatorInputsAutoLogged sensors) {
        sensors.height = getHeight();
        sensors.angle = Degrees.of(elevatorPivotMotor.getAbsoluteEncoder().getPosition() * -0.173 + 5.78).minus(Radians.of(0.1));
        sensors.horizontalSpeed = RPM.of(elevatorPivotMotor.getEncoder().getVelocity());
        sensors.extensionSpeed = InchesPerSecond.of(elevatorExtensionMotor.get());
        sensors.elevatorVoltage = Volts.of(extensionSensor.getVoltage());
        sensors.elevatorVoltageFromADC = String.format("%04x", extensionSensor.getValue());
        sensors.extension = Centimeters.of((getSensors().elevatorVoltage.in(Volts)*35.86311)+32.17113);
        sensors.canGoDown = canGoDown();
        sensors.extensionMotorPosition = Rotations.of(elevatorExtensionMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("ElevatorAngleDegrees", sensors.angle.in(Degrees));

    }

    @Override
    public void moveVertically(LinearVelocity speed) {
        elevatorExtensionMotor.set(speed.in(FeetPerSecond));
    }

    public boolean canGoDown() {
        return elevatorExtensionMotor.getReverseLimitSwitch().isPressed();
    }


    public boolean canGoUp() {
        return getSensors().elevatorVoltage.lt(Volts.of(4.248));
    }

    public boolean canGoRight() {
        return getSensors().angle.in(Degrees) > 9.55;
    }

    public boolean canGoLeft() {
        return getSensors().angle.in(Degrees) < 52.42;
    }


    @Override
    public void moveHorizontally(AngularVelocity speed) {
        elevatorPivotMotor.set(speed.in(DegreesPerSecond));
    }

    @Override
    public Distance getPivotHeight() {
        return Centimeters.of(8);
    }


}
