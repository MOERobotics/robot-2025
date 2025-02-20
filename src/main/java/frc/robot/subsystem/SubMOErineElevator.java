package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.ElevatorControl;
import frc.robot.subsystem.interfaces.ElevatorInputsAutoLogged;

import static edu.wpi.first.units.Units.*;

public class SubMOErineElevator extends MOESubsystem<ElevatorInputsAutoLogged> implements ElevatorControl {

    public SparkMax elevatorExtensionMotor;
    public SparkMax elevatorPivotMotor;
    public CANcoder tiltEncoder;
    public Distance targetHeight;
    public AnalogInput extensionSensor;

    public LinearVelocity maxExtensionSpeed = InchesPerSecond.of(6);
    PIDController pid = new PIDController(0.2, 0.2, 0);

    public SubMOErineElevator(
        SparkMax elevatorExtensionMotor,
        SparkMax elevatorPivotMotor,
        CANcoder tiltEncoder,
        AnalogInput extensionSensor
    ) {
        super(new ElevatorInputsAutoLogged());
        pid.setIntegratorRange(-0.5, 0.5);
        pid.setIZone(1.5);
        this.elevatorExtensionMotor = elevatorExtensionMotor;
        SparkMaxConfig extensionMotorConfig = new SparkMaxConfig();
        SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
        this.elevatorPivotMotor = elevatorPivotMotor;
        this.tiltEncoder = tiltEncoder;
        this.extensionSensor = extensionSensor;
        extensionMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);
        extensionMotorConfig.limitSwitch.reverseLimitSwitchEnabled(false);
        pivotMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);
        elevatorPivotMotor.configure(pivotMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        elevatorExtensionMotor.configure(extensionMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    @Override
    public void readSensors(ElevatorInputsAutoLogged sensors) {
        sensors.angle = tiltEncoder.getAbsolutePosition().getValue();
        sensors.height = getHeight();
        sensors.angle = Degrees.of(elevatorPivotMotor.getAbsoluteEncoder().getPosition() * -0.173 + 5.78).minus(Radians.of(0.1));
        sensors.horizontalSpeed = RPM.of(elevatorPivotMotor.getEncoder().getVelocity());
        sensors.extensionSpeed = InchesPerSecond.of(elevatorExtensionMotor.getEncoder().getVelocity());
        sensors.elevatorVoltage =extensionSensor.getVoltage() / .3 ; //Todo: WHYYYYYYYY
        sensors.elevatorVoltageFromADC = String.format("%04x", extensionSensor.getValue());
        sensors.extension = getExtension();
        sensors.canGoDown = canGoDown();
        sensors.extensionMotorPosition = Rotations.of(elevatorExtensionMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("ExtensionDistance",getExtension().in(Centimeters));
        SmartDashboard.putNumber("ElevatorAngleDegrees",getSensors().angle.in(Degrees) * -0.173 + 5.78);
    }

    @Override
    public void periodic() {
        if(targetHeight != null) {
            Distance error = targetHeight.minus(this.getHeight());
            double pidOutput = pid.calculate(error.in(Inches));
            pidOutput = MathUtil.clamp(pidOutput, -1, 1);
            LinearVelocity pidSpeed = maxExtensionSpeed.times(-pidOutput);
            this.moveVerticallyInner(pidSpeed);
        } else {
            targetHeight = this.getHeight();
        }
    }

    @Override
    public void setTargetHeight(Distance targetHeight) {
        this.targetHeight = targetHeight;
    }

    private void moveVerticallyInner(LinearVelocity speed){
        elevatorExtensionMotor.set(speed.in(FeetPerSecond));
    }

    @Override
    public void moveVertically(LinearVelocity speed) {
        moveVerticallyInner(speed);
        this.targetHeight = null;
    }

    @Override
    public Distance getExtension() {
        return Centimeters.of((getSensors().elevatorVoltage*41.59222)+13.11311);
    }



    public boolean canGoDown() {
        return  elevatorExtensionMotor.getReverseLimitSwitch().isPressed();
    }


    public boolean canGoUp() {
        return Volts.of(getSensors().elevatorVoltage).lt(Volts.of(4.248));
    }

    public boolean canGoRight() {
        return getSensors().angle.in(Degrees) > 9.55;
    }

    public boolean canGoLeft() {
        return getSensors().angle.in(Degrees) < 52.42;
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


}
