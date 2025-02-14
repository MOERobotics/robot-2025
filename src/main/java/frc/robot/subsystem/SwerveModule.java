package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.utils.FeedforwardConstants;
import frc.robot.utils.PIDConstants;
import frc.robot.MOESubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class SwerveModule extends MOESubsystem<SwerveModuleInputsAutoLogged> implements SwerveModuleControl{
    public SparkMax driveMotor;
    public SparkMax pivotMotor;
    public PIDController pivotController;
    public PIDController velocityDriveController;
    public CANcoder pivotEncoder;
    public Distance xPos;
    public Distance yPos;
    public Angle offset;
    public Angle targetHeading;
    SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();
    public SimpleMotorFeedforward feedforwardDrive;

    @AutoLog
    public static class SwerveModuleInputs {
        public double currentRotationDegrees;
        public double getCurrentRotationDegreesNotWrapped;
        public double pivotPower;
        public double drivePower;
        public double error, integral;
        public double pivotVolts;
        public double driveVolts;
        public double pivotVelocity;
        public double driveSpeedDesired;
        public double driveLocation;
        public double driveVelocity;
    }

    public SwerveModule(
        SparkMax driveMotor,
        SparkMax pivotMotor,
        boolean pivotInvert,
        CANcoder pivotEncoder,
        Distance xPos,
        Distance yPos,
        Angle offset,
        PIDConstants pivotFeedback,
        PIDConstants driveFeedback,
        FeedforwardConstants driveFeedforward
//            SimpleMotorFeedforward feedForwardDrive
    ) {
        this.setSensors(new SwerveModuleInputsAutoLogged());
        this.pivotEncoder = pivotEncoder;
        this.pivotMotor = pivotMotor;
        this.driveMotor = driveMotor;
        this.velocityDriveController = driveFeedback.makeController();
        this.pivotController = pivotFeedback.makeController();
        this.pivotController.enableContinuousInput(-Math.PI, Math.PI);
        this.feedforwardDrive = driveFeedforward.makeSimpleFeedforward();

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake);
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.inverted(pivotInvert).smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake);

        this.xPos = xPos;
        this.yPos = yPos;
        this.offset = offset;
        driveMotor.configure(driveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        pivotMotor.configure(pivotConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);


    }

    public Angle getHeading() {
        Angle heading = this.pivotEncoder.getAbsolutePosition().getValue().plus(offset);
        heading = Radians.of(MathUtil.angleModulus(heading.in(Radians)));
//        while (heading.gt(Rotations.of(.5))) {
//            heading = heading.minus(Rotations.of(1));
//        }
//        while (heading.lt(Rotations.of(-.5))) {
//            heading = heading.plus(Rotations.of(1));
//        }
        return heading;
    }

    @Override
    public void readSensors(SwerveModuleInputsAutoLogged sensors) {
        sensors.currentRotationDegrees = getHeading();
        sensors.pivotPower = pivotMotor.get();
        sensors.drivePower = driveMotor.get();
        sensors.targetHeading = targetHeading;
        sensors.getCurrentRotationDegreesNotWrapped = pivotMotor.getEncoder().getPosition();
        sensors.pivotVelocity = pivotMotor.getEncoder().getVelocity();
        sensors.pivotVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        sensors.driveVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        sensors.driveLocation = driveMotor.getEncoder().getPosition();
        sensors.driveVelocity = driveMotor.getEncoder().getVelocity();

    }

    public void drive(double speedMetersPerSec) {
        inputs.driveSpeedDesired = speedMetersPerSec;
        double motorPower = feedforwardDrive.calculate(speedMetersPerSec * 39.37 / 1.413);
        motorPower += velocityDriveController.calculate(inputs.driveVelocity, speedMetersPerSec * 39.37 / 1.413);
        driveMotor.setVoltage(motorPower);
        Logger.recordOutput("VelocityError", velocityDriveController.getError());
    }

    @Override
    public void pivot(Angle targetHeading) {
        this.targetHeading = targetHeading;
        Angle currentHeading = getHeading();
        Angle error = currentHeading.minus(targetHeading);
        double power = pivotController.calculate(this.getSensors().error = error.in(Radians));
        this.getSensors().integral = pivotController.getAccumulatedError();
        pivotMotor.set(power);
    }

    public void setModuleState(SwerveModuleState state) {
        state.optimize(new Rotation2d(getHeading()));
        drive(state.speedMetersPerSecond);
        pivot(state.angle.getMeasure());
    }

    public void pivotVolts(double volts) {
        pivotMotor.setVoltage(volts);
    }

    @Override
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            driveMotor.getEncoder().getVelocity(),
            new Rotation2d(getHeading())
        );
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        SwerveModulePosition position = new SwerveModulePosition(
            inchesToMeters(driveMotor.getEncoder().getPosition() * (4 * Math.PI / 6.75)),
            new Rotation2d(getHeading())
        );
        return position;
    }

}