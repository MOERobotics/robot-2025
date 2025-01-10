package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;

public class SwerveModule {

     CANcoder encoder;
     SparkMax driveMotor;
     SparkMax pivotMotor;

     PIDController pivotController;

    public double xLocation;
    public double yLocation;

    public double heading;

    public static class Inputs {
        public double getangle;
        public double pivpower;
        public double drivepower;




    }

    public SwerveModule(CANcoder encoder, SparkMax driveMotor, SparkMax pivotMotor, double xLocation, double yLocation, double heading){
        this.encoder = encoder;
        this.driveMotor = driveMotor;
        this.pivotMotor = pivotMotor;
        this.xLocation=xLocation;
        this.yLocation=yLocation;
        this.heading=heading;
        this.pivotController=new PIDController(0.001,0.001,0);
    }

    public void setDriveMotor(double speed){
        driveMotor.set(speed);
    }

    public void setPivotMotor(Angle targetHeading){
        Angle currentHeading =encoder.getAbsolutePosition().getValue();
        Angle error = currentHeading.minus(targetHeading);
        double pivotPower=  pivotController.calculate(error.in(Degrees));
        pivotMotor.set(pivotPower);
    }

}

