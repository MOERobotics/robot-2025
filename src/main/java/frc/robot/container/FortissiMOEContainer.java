package frc.robot.container;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystem.SwerveDrive;
import frc.robot.subsystem.SwerveModule;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public class FortissiMOEContainer extends RobotContainer {
    public FortissiMOEContainer () {
        this.setSwerveDrive(new SwerveDrive(
                new SwerveModule(
                        new SparkMax(17, SparkLowLevel.MotorType.kBrushless),
                        new SparkMax(16, SparkLowLevel.MotorType.kBrushless),
                        new CANcoder(34),
                        Inches.of(14),
                        Inches.of(14),
                        Degrees.of(-45)
                ),
                new SwerveModule(
                        new SparkMax(3, SparkLowLevel.MotorType.kBrushless),
                        new SparkMax(2, SparkLowLevel.MotorType.kBrushless),
                        new CANcoder(33),
                        Inches.of(-14),
                        Inches.of(14),
                        Degrees.of(-45)
                ),
                new SwerveModule(
                        new SparkMax(1, SparkLowLevel.MotorType.kBrushless),
                        new SparkMax(20, SparkLowLevel.MotorType.kBrushless),
                        new CANcoder(32),
                        Inches.of(14),
                        Inches.of(-14),
                        Degrees.of(-135)
                ),
                new SwerveModule(
                        new SparkMax(19, SparkLowLevel.MotorType.kBrushless),
                        new SparkMax(18, SparkLowLevel.MotorType.kBrushless),
                        new CANcoder(31),
                        Inches.of(-14),
                        Inches.of(14),
                        Degrees.of(135)
                ),
                new Pigeon2(0)
        ));
    }
}