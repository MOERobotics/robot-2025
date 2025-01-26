package frc.robot.container;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import frc.robot.subsystem.Elevator;
import frc.robot.subsystem.SwerveDrive;
import frc.robot.subsystem.SwerveModule;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public class SubMOErine extends RobotContainer{
    public SubMOErine(){
        SwerveModule swerveModuleFL = new SwerveModule(
                new SparkMax(1, SparkLowLevel.MotorType.kBrushless),
                new SparkMax(20, SparkLowLevel.MotorType.kBrushless),
                new CANcoder(31),
                Inches.of(14.5),
                Inches.of(14.5),
                Degrees.of(45)
        );
        SwerveModule swerveModuleFR = new SwerveModule(
                new SparkMax(3, SparkLowLevel.MotorType.kBrushless),
                new SparkMax(2, SparkLowLevel.MotorType.kBrushless),
                new CANcoder(32),
                Inches.of(14.5),
                Inches.of(-14.5),
                Degrees.of(-45)
        );
        SwerveModule swerveModuleBR = new SwerveModule(
                new SparkMax(17, SparkLowLevel.MotorType.kBrushless),
                new SparkMax(16, SparkLowLevel.MotorType.kBrushless),
                new CANcoder(33),
                Inches.of(-14.5),
                Inches.of(-14.5),
                Degrees.of(-135)
        );
        SwerveModule swerveModuleBL = new SwerveModule(
                new SparkMax(19, SparkLowLevel.MotorType.kBrushless),
                new SparkMax(18, SparkLowLevel.MotorType.kBrushless),
                new CANcoder(34),
                Inches.of(-14.5),
                Inches.of(-14.5),
                Degrees.of(-135)
        );
        Pigeon2 pigeon2 = new Pigeon2(0);
        SwerveDrive swerveDrive = new SwerveDrive(
                swerveModuleFL,
                swerveModuleFR,
                swerveModuleBR,
                swerveModuleBL,
                pigeon2
        );

        SparkMax height = new SparkMax(5, SparkLowLevel.MotorType.kBrushless);
        SparkMax pivot = new SparkMax(6, SparkLowLevel.MotorType.kBrushless);
        CANcoder tilt = new CANcoder(35);

        this.setSwerveDrive(swerveDrive);
        this.setElevator(new Elevator(
                height,
                pivot,
                tilt
        ));
    }
}


