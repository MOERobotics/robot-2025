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

import static edu.wpi.first.units.Units.Degrees;

public class SwerveModule {
    public SparkMax drivemotor;
    public SparkMax pivmotor;
    public PIDController pivotcontroller;
    public CANcoder compass;
    public Distance xpos;
    public Distance ypos;
    public Angle heading;

    public static class Inputs {
        public double getangle;
        public double pivpower;
        public double drivepower;

    }

    public SwerveModule(
            SparkMax drivemotor,
            SparkMax pivmotor,
            CANcoder compass,
            Distance xpos,
            Distance ypos,
            Angle heading
    ) {
        this.compass = compass;
        this.pivmotor = pivmotor;
        this.drivemotor = drivemotor;
        this.pivotcontroller = new PIDController(0.001, 0.001, 0);
        this.xpos = xpos;
        this.ypos = ypos;
        this.heading = heading;

    }

    public void drive(double power) {
        drivemotor.set(power);
    }

    public void pivot(Angle targetHeading) {
        targetHeading = targetHeading.plus(heading);
        Angle currentHeading = compass.getAbsolutePosition().getValue();
        Angle error = currentHeading.minus(targetHeading);
        double pivpower = pivotcontroller.calculate(error.in(Degrees));
        pivmotor.set(pivpower);
    }

    public SwerveModuleState getmodulestate() {
        return new SwerveModuleState(
                drivemotor.getEncoder().getVelocity(),
                new Rotation2d(compass.getAbsolutePosition().getValue())
        );
    }

    public SwerveModulePosition getModulePosition() {
        SwerveModulePosition position = new SwerveModulePosition(
                Units.Inches.of(drivemotor.getEncoder().getPosition()).in(Units.Meters),
                new Rotation2d(compass.getAbsolutePosition().getValue())
        );
        return position;

    }
}