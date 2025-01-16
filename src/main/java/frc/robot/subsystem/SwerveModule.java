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
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Degrees;

public class SwerveModule {
    public SparkMax driveMotor;
    public SparkMax pivotMotor;
    public PIDController pivotController;
    public CANcoder compass;
    public Distance xPos;
    public Distance yPos;
    public Angle heading;

    @AutoLog
    public static class SwerveModuleInputs {
        public double currentRotationDegrees;
        public double pivotPower;
        public double drivePower;

    }

    public SwerveModule(
            SparkMax driveMotor,
            SparkMax pivotMotor,
            CANcoder compass,
            Distance xPos,
            Distance yPos,
            Angle heading
    ) {
        this.compass = compass;
        this.pivotMotor = pivotMotor;
        this.driveMotor = driveMotor;
        this.pivotController = new PIDController(0.001, 0.001, 0);
        this.xPos = xPos;
        this.yPos = yPos;
        this.heading = heading;

    }

    public void readSensors(SwerveModuleInputsAutoLogged inputs) {
        inputs.currentRotationDegrees = this.compass.getAbsolutePosition().getValue().in(Degrees);
        inputs.pivotPower = pivotMotor.get();
        inputs.drivePower = driveMotor.get();
    }

    public void drive(double power) {
        driveMotor.set(power);
    }

    public void pivot(Angle targetHeading) {
        targetHeading = targetHeading.plus(heading);
        Angle currentHeading = compass.getAbsolutePosition().getValue();
        Angle error = currentHeading.minus(targetHeading);
        double power = pivotController.calculate(error.in(Degrees));
        pivotMotor.set(power);
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
                driveMotor.getEncoder().getVelocity(),
                new Rotation2d(compass.getAbsolutePosition().getValue())
        );
    }

    public SwerveModulePosition getModulePosition() {
        SwerveModulePosition position = new SwerveModulePosition(
                Units.Inches.of(driveMotor.getEncoder().getPosition()).in(Units.Meters),
                new Rotation2d(compass.getAbsolutePosition().getValue())
        );
        return position;

    }
}