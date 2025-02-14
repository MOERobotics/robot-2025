package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
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
import frc.robot.utils.FeedforwardConstants;
import frc.robot.utils.PIDConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class SwerveModule {
    public SparkMax driveMotor;
    public SparkMax pivotMotor;
    public PIDController pivotController;
    public PIDController velocityDriveController;
    public CANcoder pivotEncoder;
    public Distance xPos;
    public Distance yPos;
    public Angle offset;
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

    public SwerveModuleInputsAutoLogged readSensors() {
        inputs.currentRotationDegrees = getHeading().in(Degrees);
        inputs.getCurrentRotationDegreesNotWrapped = pivotMotor.getEncoder().getPosition();
        inputs.pivotPower = pivotMotor.get();
        inputs.drivePower = driveMotor.get();
        inputs.pivotVelocity = pivotMotor.getEncoder().getVelocity();
        inputs.pivotVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.driveVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.driveLocation = driveMotor.getEncoder().getPosition();
        inputs.driveVelocity = driveMotor.getEncoder().getVelocity();

        return inputs;
    }

    public void drive(double speedMetersPerSec) {
        inputs.driveSpeedDesired = speedMetersPerSec;
        double motorPower = feedforwardDrive.calculate(speedMetersPerSec * 39.37 / 1.413);
        motorPower += velocityDriveController.calculate(inputs.driveVelocity, speedMetersPerSec * 39.37 / 1.413);
        driveMotor.setVoltage(motorPower);
        Logger.recordOutput("VelocityError", velocityDriveController.getError());
    }

    public void pivot(Angle targetHeading) {
        Angle currentHeading = getHeading();
        Angle error = currentHeading.minus(targetHeading);
        double power = pivotController.calculate(inputs.error = error.in(Radians));
        inputs.integral = pivotController.getAccumulatedError();
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

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            driveMotor.getEncoder().getVelocity(),
            new Rotation2d(getHeading())
        );
    }

    public SwerveModulePosition getModulePosition() {
        SwerveModulePosition position = new SwerveModulePosition(
            inchesToMeters(driveMotor.getEncoder().getPosition() * (4 * Math.PI / 6.75)),
            new Rotation2d(getHeading())
        );
        return position;
    }

}