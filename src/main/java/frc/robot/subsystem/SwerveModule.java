package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkClosedLoopController;
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
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class SwerveModule {
    public SparkMax driveMotor;
    public SparkMax pivotMotor;
    public PIDController pivotController;
    public PIDController velocityDriveController = new PIDController(2e-3,0,0);
    public CANcoder compass;
    public Distance xPos;
    public Distance yPos;
    public Angle heading;
    SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();
    public SimpleMotorFeedforward feedforwardDrive = new SimpleMotorFeedforward(0.19959,0.1233,0.019658);
    public SimpleMotorFeedforward feedforwardPivot = new SimpleMotorFeedforward(0.025*0,0.0,0);

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
            CANcoder compass,
            Distance xPos,
            Distance yPos,
            Angle heading,
            PIDController pivotController
    ) {
        this.compass = compass;
        this.pivotMotor = pivotMotor;
        this.driveMotor = driveMotor;
        this.pivotController = pivotController;
        this.xPos = xPos;
        this.yPos = yPos;
        this.heading = heading;


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
        double motorPower = feedforwardDrive.calculate(speedMetersPerSec*39.37/1.413);
        motorPower += velocityDriveController.calculate(inputs.driveVelocity,speedMetersPerSec*39.37/1.413);
        driveMotor.setVoltage(motorPower);
        Logger.recordOutput("VelocityError", velocityDriveController.getError());
    }

    public void pivot(Angle targetHeading) {
        Angle currentHeading = getHeading();
        Angle error = currentHeading.minus(targetHeading);
        double power = feedforwardPivot.calculate(-error.in(Degrees));
        power += pivotController.calculate(inputs.error = error.in(Radians));
        inputs.integral = pivotController.getAccumulatedError();
        pivotMotor.set(power);
    }

    public void setModuleState(SwerveModuleState state){
        state.optimize(new Rotation2d(getHeading()));
        drive(state.speedMetersPerSecond);
        pivot(state.angle.getMeasure());
    }

    public void pivotVolts(double volts){
        pivotMotor.setVoltage(volts);
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