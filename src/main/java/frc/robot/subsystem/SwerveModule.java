package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.SwerveModuleControl;
import frc.robot.subsystem.interfaces.SwerveModuleInputsAutoLogged;

import static edu.wpi.first.units.Units.*;

public class SwerveModule extends MOESubsystem<SwerveModuleInputsAutoLogged> implements SwerveModuleControl {
    public SparkMax driveMotor;
    public SparkMax pivotMotor;
    public PIDController pivotController;
    public CANcoder pivotEncoder;
    public Distance xPos;
    public Distance yPos;
    public Angle moduleOffset;
    public Angle targetHeading;

    public SwerveModule(
            SparkMax driveMotor,
            SparkMax pivotMotor,
            CANcoder pivotEncoder,
            Distance xPos,
            Distance yPos,
            Angle moduleOffset,
            PIDController pivotController
    ) {
        super(new SwerveModuleInputsAutoLogged());
        this.pivotEncoder = pivotEncoder;
        this.pivotMotor = pivotMotor;
        this.driveMotor = driveMotor;
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        driveConfig.inverted(false).idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);
        pivotConfig.inverted(true).idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(30);
        this.pivotController = pivotController;
        pivotController.enableContinuousInput(-Math.PI,Math.PI);
        this.xPos = xPos;
        this.yPos = yPos;
        this.moduleOffset = moduleOffset;
        driveMotor.configure(driveConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        pivotMotor.configure(pivotConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);


    }

    public Angle getModuleOffset() {
        Angle _heading =  this.pivotEncoder.getAbsolutePosition().getValue().plus(moduleOffset);
        while (_heading.gt(Rotations.of(.5))) {
            _heading = _heading.minus(Rotations.of(1));
        }
        while (_heading.lt(Rotations.of(-.5))) {
            _heading = _heading.plus(Rotations.of(1));
        }
        return _heading;
    }

    @Override
    public void readSensors(SwerveModuleInputsAutoLogged sensors) {
        sensors.currentRotationDegrees =  getModuleOffset();
        sensors.pivotPower = pivotMotor.get();
        sensors.drivePower = driveMotor.get();
        sensors.targetHeading = targetHeading;
    }

    @Override
    public void drive(double power) {
        driveMotor.set(power);
    }

    @Override
    public void pivot(Angle targetHeading) {
        this.targetHeading = targetHeading;
        Angle currentHeading = getModuleOffset();
        Angle error = currentHeading.minus(targetHeading);
        double power = pivotController.calculate(this.getSensors().error = error.in(Radians));
        this.getSensors().integral = pivotController.getAccumulatedError();
        pivotMotor.set(power);
    }

    @Override
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
                driveMotor.getEncoder().getVelocity(),
                new Rotation2d( getModuleOffset())
        );
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        SwerveModulePosition position = new SwerveModulePosition(
                Units.Inches.of(driveMotor.getEncoder().getPosition()).in(Units.Meters),
                new Rotation2d( getModuleOffset())
        );
        return position;

    }

    @Override
    public void setModuleState(SwerveModuleState moduleState) {
        moduleState.optimize(Rotation2d.fromDegrees(getModuleOffset().in(Degrees)));
        drive(moduleState.speedMetersPerSecond);
        pivot( moduleState.angle.getMeasure());
    }
}