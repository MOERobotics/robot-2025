package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.SwerveModuleControl;
import frc.robot.subsystem.interfaces.SwerveModuleInputsAutoLogged;
import frc.robot.utils.FeedforwardConstants;
import frc.robot.utils.PIDConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.MathUtil.*;
import static edu.wpi.first.units.Units.*;

public class SwerveModule extends MOESubsystem<SwerveModuleInputsAutoLogged> implements SwerveModuleControl {
    public SparkMax driveMotor;
    public SparkMax pivotMotor;
    public PIDController pivotController;
    public PIDController velocityDriveController;
    public CANcoder pivotEncoder;
    public Distance xPos;
    public Distance yPos;
    public Angle moduleOffset;
    public Angle targetHeading;
    public SimpleMotorFeedforward driveFeedforward;

    public SwerveModule(
        SparkMax driveMotor,
        SparkMax pivotMotor,
        boolean driveInvert,
        boolean pivotInvert,
        CANcoder pivotEncoder,
        Distance xPos,
        Distance yPos,
        Angle moduleOffset,
        PIDConstants pivotFeedback,
        PIDConstants driveFeedback,
        FeedforwardConstants driveFeedforward
    ) {
        super(new SwerveModuleInputsAutoLogged());
        this.pivotEncoder = pivotEncoder;
        this.pivotMotor = pivotMotor;
        this.driveMotor = driveMotor;
        this.velocityDriveController = driveFeedback.makeController();
        this.pivotController = pivotFeedback.makeController();
        this.pivotController.enableContinuousInput(-Math.PI, Math.PI);
        this.driveFeedforward = driveFeedforward.makeSimpleFeedforward();

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.inverted(driveInvert).smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake);
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.inverted(pivotInvert).smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake);

        this.xPos = xPos;
        this.yPos = yPos;
        this.moduleOffset = moduleOffset;
        driveMotor.configure(driveConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        pivotMotor.configure(pivotConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);


    }

    public Angle getHeading() {
        Angle heading = this.pivotEncoder.getAbsolutePosition().getValue().plus(moduleOffset);
        heading = Radians.of(MathUtil.angleModulus(heading.in(Radians)));
        getSensors().currentRotationDegrees = heading;
        return heading;
    }

    @Override
    public void readSensors(SwerveModuleInputsAutoLogged sensors) {
        sensors.pivotPower = pivotMotor.get();
        sensors.drivePower = driveMotor.get();
        sensors.currentRotationDegreesNotWrapped = pivotEncoder.getAbsolutePosition().getValue();
        sensors.pivotVelocity = RPM.of(pivotMotor.getEncoder().getVelocity());
        sensors.wheelPivotVelocity = pivotEncoder.getVelocity().getValue();
        sensors.pivotVolts = Volts.of(pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage());
        sensors.driveVolts = Volts.of(driveMotor.getAppliedOutput() * driveMotor.getBusVoltage());
        sensors.drivePosition = Rotations.of(driveMotor.getEncoder().getPosition());
        sensors.driveVelocity = RPM.of(driveMotor.getEncoder().getVelocity());
    }


    @Override
    public void drive(double speedMetersPerSec) {
        getSensors().driveSpeedDesired = MetersPerSecond.of(speedMetersPerSec);
        double motorPower = driveFeedforward.calculate(speedMetersPerSec * 39.37 * 60 / (4*Math.PI/6.75));
        motorPower += velocityDriveController.calculate(getSensors().driveVelocity.in(RPM), speedMetersPerSec * 39.37 * 60 / (4*Math.PI/6.75));
        driveMotor.set(motorPower/driveMotor.getBusVoltage());
        Logger.recordOutput("VelocityError", velocityDriveController.getError());
    }

    @Override
    public void pivot(Angle targetHeading) {
        getSensors().targetHeading= targetHeading;
        Angle currentHeading = getHeading();
        getSensors().error = currentHeading.minus(targetHeading).in(Radians);
        double power = pivotController.calculate(currentHeading.in(Radians),targetHeading.in(Radians));
        getSensors().integral = pivotController.getAccumulatedError();
        pivotMotor.set(clamp(power,-1,1));
    }

    @Override
    public void setModuleState(SwerveModuleState moduleState) {
        moduleState.optimize(new Rotation2d(getHeading()));
        drive(moduleState.speedMetersPerSecond);
        pivot(moduleState.angle.getMeasure());
    }

    public void pivotVolts(double volts) {
        pivotMotor.setVoltage(volts);
    }

    @Override
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
                InchesPerSecond.of(driveMotor.getEncoder().getVelocity() * (4*Math.PI/6.75) / 60).in(MetersPerSecond),
                new Rotation2d(getHeading())
        );
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        SwerveModulePosition position = new SwerveModulePosition(
            Inches.of(driveMotor.getEncoder().getPosition() * (4 * Math.PI / 6.75)).in(Meters),
            new Rotation2d(getHeading())
        );
        return position;
    }

}