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
import frc.robot.utils.FeedforwardConstants;
import frc.robot.utils.PIDConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.*;

public class SwerveModule implements SwerveModuleControl{
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
    public SimpleMotorFeedforward driveFeedforward;

    public SwerveModule(
        SparkMax driveMotor,
        SparkMax pivotMotor,
        boolean driveInvert,
        boolean pivotInvert,
        CANcoder pivotEncoder,
        Distance xPos,
        Distance yPos,
        Angle offset,
        PIDConstants pivotFeedback,
        PIDConstants driveFeedback,
        FeedforwardConstants driveFeedforward
    ) {
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
    public SwerveModuleInputsAutoLogged readSensors() {
        inputs.currentRotationDegrees = getHeading();
        inputs.pivotPower = pivotMotor.get();
        inputs.drivePower = driveMotor.get();
        inputs.targetHeading = targetHeading;
        inputs.currentRotationDegreesNotWrapped = Rotations.of(pivotMotor.getEncoder().getPosition());
        inputs.pivotVelocity = RPM.of(pivotMotor.getEncoder().getVelocity());
        inputs.pivotVolts = Volts.of(pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage());
        inputs.driveVolts = Volts.of(driveMotor.getAppliedOutput() * driveMotor.getBusVoltage());
        inputs.drivePosition = Rotations.of(driveMotor.getEncoder().getPosition());
        inputs.driveVelocity = RPM.of(driveMotor.getEncoder().getVelocity());

        return inputs;
    }


    @Override
    public void drive(double speedMetersPerSec) {
        inputs.driveSpeedDesired = MetersPerSecond.of(speedMetersPerSec);
        double motorPower = driveFeedforward.calculate(speedMetersPerSec * 39.37 / 1.413);
        motorPower += velocityDriveController.calculate(inputs.driveVelocity.in(RPM), speedMetersPerSec * 39.37 / 1.413);
        driveMotor.setVoltage(motorPower);
        Logger.recordOutput("VelocityError", velocityDriveController.getError());
    }

    @Override
    public void pivot(Angle targetHeading) {
        this.targetHeading = targetHeading;
        Angle currentHeading = getHeading();
        Angle error = currentHeading.minus(targetHeading);
        double power = pivotController.calculate(this.inputs.error = error.in(Radians));
        this.inputs.integral = pivotController.getAccumulatedError();
        pivotMotor.set(power);
    }

    @Override
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