package frc.robot.container;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.subsystem.*;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public class SubMOErine extends RobotContainer {
    public SubMOErine() {
        double pivotkP = 0.15;
        double pivotkI = 0.001;
        double pivotkD = 0.003;
        ;
        double pivotkIMax = 1;
        PIDController pivotControllerFL = new PIDController(pivotkP, pivotkI, pivotkD);
        pivotControllerFL.setIntegratorRange(-pivotkIMax, pivotkIMax);
        pivotControllerFL.enableContinuousInput(-Math.PI, Math.PI);
        SwerveModule swerveModuleFL = new SwerveModule(
            new SparkMax(1, SparkLowLevel.MotorType.kBrushless),
            new SparkMax(20, SparkLowLevel.MotorType.kBrushless),
            new CANcoder(31),
            Inches.of(14.5),
            Inches.of(14.5),
            Degrees.of(45),
            pivotControllerFL
        );
        PIDController pivotControllerFR = new PIDController(pivotkP, pivotkI, pivotkD);
        pivotControllerFR.setIntegratorRange(-pivotkIMax, pivotkIMax);
        pivotControllerFR.enableContinuousInput(-Math.PI, Math.PI);
        SwerveModule swerveModuleFR = new SwerveModule(
            new SparkMax(3, SparkLowLevel.MotorType.kBrushless),
            new SparkMax(2, SparkLowLevel.MotorType.kBrushless),
            new CANcoder(32),
            Inches.of(14.5),
            Inches.of(-14.5),
            Degrees.of(-45),
            pivotControllerFR
        );
        PIDController pivotControllerBR = new PIDController(pivotkP, pivotkI, pivotkD);
        pivotControllerBR.setIntegratorRange(-pivotkIMax, pivotkIMax);
        pivotControllerBR.enableContinuousInput(-Math.PI, Math.PI);
        SwerveModule swerveModuleBR = new SwerveModule(
            new SparkMax(17, SparkLowLevel.MotorType.kBrushless),
            new SparkMax(16, SparkLowLevel.MotorType.kBrushless),
            new CANcoder(33),
            Inches.of(-14.5),
            Inches.of(-14.5),
            Degrees.of(-135),
            pivotControllerBR
        );
        PIDController pivotControllerBL = new PIDController(pivotkP, pivotkI, pivotkD);
        pivotControllerBL.setIntegratorRange(-pivotkIMax, pivotkIMax);
        pivotControllerBL.enableContinuousInput(-Math.PI, Math.PI);
        SwerveModule swerveModuleBL = new SwerveModule(
            new SparkMax(19, SparkLowLevel.MotorType.kBrushless),
            new SparkMax(18, SparkLowLevel.MotorType.kBrushless),
            new CANcoder(34),
            Inches.of(-14.5),
            Inches.of(14.5),
            Degrees.of(135),
            pivotControllerBL
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

        AnalogInput extensionSensor = new AnalogInput(1);


        this.setSwerveDrive(swerveDrive);
        this.setElevator(new SubMOErineElevator(
            height,
            pivot,
            tilt,
            extensionSensor
        ));

        this.setCoralCollector(new CoralCollector(new SparkMax(13, SparkLowLevel.MotorType.kBrushless), new SparkMax(12, SparkLowLevel.MotorType.kBrushless)));
        this.setAlgaeCollector(new AlgaeCollector(
            new SparkMax(11, SparkLowLevel.MotorType.kBrushless),
            new SparkMax(10, SparkLowLevel.MotorType.kBrushless),
            new CANcoder(36),
            Degrees.of(90),
            Degrees.of(0),
            Degrees.of(1)
        ));

        this.setClimberMid(new SubMOErineClimber(
            new SparkMax(7, SparkLowLevel.MotorType.kBrushless),
            new CANcoder(37)
        ));

        this.setClimberRear(new SubMOErineClimber(
            new SparkMax(8, SparkLowLevel.MotorType.kBrushless),
            new CANcoder(38)
        ));

    }
}


