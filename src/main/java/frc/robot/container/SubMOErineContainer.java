package frc.robot.container;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import frc.robot.subsystem.CoralCollector;
import frc.robot.subsystem.CoralCollector.CoralCollectorConstants;
import frc.robot.subsystem.SwerveDrive;
import frc.robot.subsystem.SwerveModule;

import static edu.wpi.first.units.Units.*;

public class SubMOErineContainer extends RobotContainer{
    public SubMOErineContainer(){
        this.setSwerveDrive(new SwerveDrive(
            new SwerveModule(
                new SparkMax(1, SparkLowLevel.MotorType.kBrushless),
                new SparkMax(20, SparkLowLevel.MotorType.kBrushless),
                new CANcoder(31),
                Inches.of(14.5),
                Inches.of(14.5),
                Degrees.of(45)
            ),
            new SwerveModule(
                new SparkMax(3, SparkLowLevel.MotorType.kBrushless),
                new SparkMax(2, SparkLowLevel.MotorType.kBrushless),
                new CANcoder(32),
                Inches.of(14.5),
                Inches.of(-14.5),
                Degrees.of(-45)
            ),
            new SwerveModule(
                new SparkMax(17, SparkLowLevel.MotorType.kBrushless),
                new SparkMax(16, SparkLowLevel.MotorType.kBrushless),
                new CANcoder(33),
                Inches.of(-14.5),
                Inches.of(-14.5),
                Degrees.of(-135)
            ),
            new SwerveModule(
                new SparkMax(19, SparkLowLevel.MotorType.kBrushless),
                new SparkMax(18, SparkLowLevel.MotorType.kBrushless),
                new CANcoder(34),
                Inches.of(-14.5),
                Inches.of(14.5),
                Degrees.of(135)
            ),
            new Pigeon2(0)
        ));

        this.setCoralCollector(new CoralCollector(CoralCollectorConstants.builder()
            .leftMotorID(13)
            .rightMotorID(12)
            .leftInvert(false)
            .rightInvert(true)
            .currentLimit(40)
            .kP(1.0)
            .kI(0.0)
            .kD(0.0)
            .build()));

    }
}
