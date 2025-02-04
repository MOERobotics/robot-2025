package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.MOESubsystem;

import static edu.wpi.first.units.Units.*;

public class SwerveModule extends MOESubsystem<SwerveModuleInputsAutoLogged> implements SwerveModuleControl{
    public SparkMax driveMotor;
    public SparkMax pivotMotor;
    public PIDController pivotController;
    public CANcoder compass;
    public Distance xPos;
    public Distance yPos;
    public Angle heading;

    public SwerveModule(
            SparkMax driveMotor,
            SparkMax pivotMotor,
            CANcoder compass,
            Distance xPos,
            Distance yPos,
            Angle heading,
            PIDController pivotController
    ) {
        this.setSensors(new SwerveModuleInputsAutoLogged());
        this.compass = compass;
        this.pivotMotor = pivotMotor;
        this.driveMotor = driveMotor;
        this.pivotController = pivotController;
        pivotController.setTolerance(Math.PI/180);
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

    @Override
    public void readSensors(SwerveModuleInputsAutoLogged sensors) {
        sensors.currentRotationDegrees =  getHeading();
        sensors.pivotPower = pivotMotor.get();
        sensors.drivePower = driveMotor.get();
    }

    @Override
    public void drive(double power) {
        driveMotor.set(power/4);
    }

    @Override
    public void pivot(Angle targetHeading) {
        Angle currentHeading = getHeading();
        Angle error = currentHeading.minus(targetHeading);
        double power = pivotController.calculate(this.getSensors().error = error.in(Radians));
        this.getSensors().integral = pivotController.getAccumulatedError();
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