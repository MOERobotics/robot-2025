package frc.robot.container;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.LidarTest;
import frc.robot.subsystem.FakeElevator;
import frc.robot.subsystem.FakeCoralCollector;
import frc.robot.subsystem.SwerveDrive;
import frc.robot.subsystem.SwerveModule;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
//hello2

public class SwerveBotContainer extends RobotContainer {

    public SwerveBotContainer (){
        double pivotkP = 8.0e-3*60;
        double pivotkI = 0;
        double pivotkD = 0;
        double pivotkIMax = 2;
        double driveP = 7.0e-5;
        double driveI = 0.0;
        double driveD = 1.0e-4;
        //double driveFF = 1.76182e-4;
        PIDController pivotControllerFL = new PIDController(pivotkP, pivotkI, pivotkD);
        pivotControllerFL.setIntegratorRange(-pivotkIMax, pivotkIMax);
        PIDController pivotControllerFR = new PIDController(pivotkP, pivotkI, pivotkD);
        pivotControllerFR.setIntegratorRange(-pivotkIMax, pivotkIMax);
        PIDController pivotControllerBR = new PIDController(pivotkP, pivotkI, pivotkD);
        pivotControllerBR.setIntegratorRange(-pivotkIMax, pivotkIMax);
        PIDController pivotControllerBL = new PIDController(pivotkP, pivotkI, pivotkD);
        pivotControllerBL.setIntegratorRange(-pivotkIMax, pivotkIMax);
        PIDController driveControllerFL = new PIDController(driveD, driveI, driveD);
        PIDController driveControllerFR = new PIDController(driveD, driveI, driveD);
        PIDController driveControllerBL = new PIDController(driveD, driveI, driveD);
        PIDController driveControllerBR = new PIDController(driveD, driveI, driveD);
        SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0,1.76182e-4,0);

        this.setSwerveDrive(
                new SwerveDrive(
                        new SwerveModule(//FL
                                new SparkMax(11, SparkLowLevel.MotorType.kBrushless),
                                new SparkMax(10, SparkLowLevel.MotorType.kBrushless){{this.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);}},
                                new CANcoder(31),
                                Inches.of(14),
                                Inches.of(14),
                                Degrees.of(45),
                                pivotControllerFL,
                                driveControllerFL,
                                driveFF
                        ),
                        new SwerveModule(//FR
                                new SparkMax(9, SparkLowLevel.MotorType.kBrushless),
                                new SparkMax(8, SparkLowLevel.MotorType.kBrushless){{this.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);}},
                                new CANcoder(32),
                                Inches.of(14),
                                Inches.of(-14),
                                Degrees.of(-45),
                                pivotControllerFR,
                                driveControllerFR,
                                driveFF
                        ),
                        new SwerveModule(//BR
                                new SparkMax( 1, SparkLowLevel.MotorType.kBrushless),
                                new SparkMax(20, SparkLowLevel.MotorType.kBrushless){{this.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);}},
                                new CANcoder(33),
                                Inches.of(-14),
                                Inches.of(-14),
                                Degrees.of(-135),
                                pivotControllerBR,
                                driveControllerBR,
                                driveFF
                        ),
                        new SwerveModule(//BL
                                new SparkMax(19, SparkLowLevel.MotorType.kBrushless),
                                new SparkMax(18, SparkLowLevel.MotorType.kBrushless){{this.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);}},
                                new CANcoder(34),
                                Inches.of(-14),
                                Inches.of(14),
                                Degrees.of(135),
                                pivotControllerBL,
                                driveControllerBL,
                                driveFF
                        ),
                        new Pigeon2(0)

                ));
        this.setElevator(new FakeElevator());

        this.setCoralCollector(new FakeCoralCollector());
    }

}
