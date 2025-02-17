package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public class SwerveModule {
    public SparkMax driveMotor;
    public SparkMax pivotMotor;
    public PIDController pivotController;

    public SimpleMotorFeedforward driveFF;

    public PIDController driveController;
    public CANcoder compass;
    public Distance xPos;
    public Distance yPos;
    public Angle heading;

    public double velocityConversionFactor;
    SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();

    @AutoLog
    public static class SwerveModuleInputs {
        public double currentRotationDegrees;
        public double pivotPower;
        public double drivePower;
        public double error, integral;

    }

    public SwerveModule(
            SparkMax driveMotor,
            SparkMax pivotMotor,
            CANcoder compass,
            Distance xPos,
            Distance yPos,
            Angle heading,
            PIDController pivotController,
            PIDController driveController,
            SimpleMotorFeedforward driveFF
    ) {
        this.compass = compass;
        this.pivotMotor = pivotMotor;
        this.driveMotor = driveMotor;
        this.pivotController = pivotController;
        this.driveController = driveController;
        this.xPos = xPos;
        this.yPos = yPos;
        this.heading = heading;
        this.driveFF = driveFF;
    }

    public Angle getHeading() {
        Angle _heading =  this.compass.getAbsolutePosition().getValue().plus(heading);
        while (_heading.gt(Rotations.of(.5))) {
            _heading = _heading.minus(Rotations.of(1));
        }
        while (_heading.lt(Rotations.of(-.5))) {
            _heading = _heading.plus(Rotations.of(1));
        }
        return _heading;
    }

    public SwerveModuleInputsAutoLogged readSensors() {
        inputs.currentRotationDegrees =  getHeading().in(Degrees);
        inputs.pivotPower = pivotMotor.get();
        inputs.drivePower = driveMotor.get();
        return inputs;
    }

    public double getDriveVelocity(){
        return driveMotor.getEncoder().getVelocity()/velocityConversionFactor;
    }

    public void drive(double target_power) {
        double currentVelocity =  getDriveVelocity();
        double power = driveController.calculate(currentVelocity, target_power);
        power = power + driveFF.calculate(target_power);
        inputs.integral = driveController.getAccumulatedError();
        driveMotor.set(power);
    }

    public void pivot(Angle targetHeading) {
        Angle currentHeading =  getHeading();
        Angle error = currentHeading.minus(targetHeading).plus(Degrees.of(180));
        double power = pivotController.calculate(inputs.error = -error.in(Radians));
        inputs.integral = pivotController.getAccumulatedError();
        pivotMotor.set(power);
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
                driveMotor.getEncoder().getVelocity(),
                new Rotation2d( getHeading())
        );
    }

    public SwerveModulePosition getModulePosition() {
        SwerveModulePosition position = new SwerveModulePosition(
                Units.Inches.of(driveMotor.getEncoder().getPosition()).in(Units.Meters),
                new Rotation2d( getHeading())
        );
        return position;

    }
}