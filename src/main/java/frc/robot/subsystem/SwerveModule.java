package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;

public class SwerveModule {
public SparkMax drivemotor;
    public SparkMax pivmotor;
    public PIDController pivotcontroller;
    public CANcoder compass;
    public double xpos;
    public double ypos;
    public double heading;
    public static class Inputs {
        public double getangle;
        public double pivpower;
        public double drivepower;

    }
    public SwerveModule(
            SparkMax drivemotor,
            SparkMax pivmotor,
            CANcoder compass,
            double xpos ,
            double ypos,
            double heading

    ){
        this.compass = compass;
        this.pivmotor = pivmotor;
        this. drivemotor = drivemotor;
        this.pivotcontroller = new PIDController(0.001,0.001,0);
        this.xpos = xpos;
        this.ypos = ypos;
        this.heading = heading;

    }
    public void drive(double power){
        drivemotor.set(power);
    }
    public void pivot(Angle targetHeading){
        Angle currentHeading = compass.getAbsolutePosition().getValue();
        Angle error = currentHeading.minus(targetHeading);
        double pivpower = pivotcontroller.calculate(error.in(Degrees));
        pivmotor.set(pivpower);
    }
}
