package frc.robot.container;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import frc.robot.subsystem.SubMOErineElevator;
import frc.robot.subsystem.SwerveDrive;
import frc.robot.subsystem.SwerveModule;
import frc.robot.utils.FeedforwardConstants;
import frc.robot.utils.PIDConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public class SubMOErine extends RobotContainer {
    public SubMOErine() {
        double pivotkP = 0.01;
        double pivotkI = 0.3;
        double pivotkD = 0;
        double pivotkIMax = 1;

        double drivekP = 1e-3;
        double drivekI = 0;
        double drivekD = 0;

        double drivekS = 0.19959;
        double drivekV = 0.1233;
        double drivekA = 0.019658;

        PIDConstants pivotFeedback = new PIDConstants(pivotkP, pivotkI, pivotkD, pivotkIMax);
        PIDConstants driveFeedback = new PIDConstants(drivekP, drivekI, drivekD);
        FeedforwardConstants driveFeedForward = new FeedforwardConstants(drivekS, drivekV, drivekA);

        SwerveModule swerveModuleFL = new SwerveModule(
            new SparkMax(1, SparkLowLevel.MotorType.kBrushless),
            new SparkMax(20, SparkLowLevel.MotorType.kBrushless),
            true,
            new CANcoder(31),
            Inches.of(14.5),
            Inches.of(14.5),
            Degrees.of(45),
            pivotFeedback,
            driveFeedback,
            driveFeedForward
        );
        SwerveModule swerveModuleFR = new SwerveModule(
            new SparkMax(3, SparkLowLevel.MotorType.kBrushless),
            new SparkMax(2, SparkLowLevel.MotorType.kBrushless),
            true,
            new CANcoder(32),
            Inches.of(14.5),
            Inches.of(-14.5),
            Degrees.of(-45),
            pivotFeedback,
            driveFeedback,
            driveFeedForward
        );
        SwerveModule swerveModuleBR = new SwerveModule(
            new SparkMax(17, SparkLowLevel.MotorType.kBrushless),
            new SparkMax(16, SparkLowLevel.MotorType.kBrushless),
            true,
            new CANcoder(33),
            Inches.of(-14.5),
            Inches.of(-14.5),
            Degrees.of(-135),
            pivotFeedback,
            driveFeedback,
            driveFeedForward
        );
        SwerveModule swerveModuleBL = new SwerveModule(
            new SparkMax(19, SparkLowLevel.MotorType.kBrushless),
            new SparkMax(18, SparkLowLevel.MotorType.kBrushless),
            true,
            new CANcoder(34),
            Inches.of(-14.5),
            Inches.of(-14.5),
            Degrees.of(-135),
            pivotFeedback,
            driveFeedback,
            driveFeedForward
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
        this.setElevator(new SubMOErineElevator(
            height,
            pivot,
            tilt
        ));
    }
}


