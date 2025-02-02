package frc.robot.container;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystem.FakeElevator;
import frc.robot.subsystem.FakeCoralCollector;
import frc.robot.subsystem.SwerveDrive;
import frc.robot.subsystem.SwerveModule;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public class SwerveBotContainer extends RobotContainer {

    public SwerveBotContainer (){
        double pivotkP = 8.0e-3*60;
        double pivotkI = 0;
        double pivotkD = 0;
        double pivotkIMax = 2;
        PIDController pivotControllerFL = new PIDController(pivotkP, pivotkI, pivotkD);
        pivotControllerFL.setIntegratorRange(-pivotkIMax, pivotkIMax);
        PIDController pivotControllerFR = new PIDController(pivotkP, pivotkI, pivotkD);
        pivotControllerFR.setIntegratorRange(-pivotkIMax, pivotkIMax);
        PIDController pivotControllerBR = new PIDController(pivotkP, pivotkI, pivotkD);
        pivotControllerBR.setIntegratorRange(-pivotkIMax, pivotkIMax);
        PIDController pivotControllerBL = new PIDController(pivotkP, pivotkI, pivotkD);
        pivotControllerBL.setIntegratorRange(-pivotkIMax, pivotkIMax);

        this.setSwerveDrive(
                new SwerveDrive(
                        new SwerveModule(//FL
                                new SparkMax(11, SparkLowLevel.MotorType.kBrushless),
                                new SparkMax(10, SparkLowLevel.MotorType.kBrushless){{this.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);}},
                                new CANcoder(31),
                                Inches.of(14),
                                Inches.of(14),
                                Degrees.of(45),
                                pivotControllerFL
                        ),
                        new SwerveModule(//FR
                                new SparkMax(9, SparkLowLevel.MotorType.kBrushless),
                                new SparkMax(8, SparkLowLevel.MotorType.kBrushless){{this.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);}},
                                new CANcoder(32),
                                Inches.of(14),
                                Inches.of(-14),
                                Degrees.of(-45),
                                pivotControllerFR
                        ),
                        new SwerveModule(//BR
                                new SparkMax( 1, SparkLowLevel.MotorType.kBrushless),
                                new SparkMax(20, SparkLowLevel.MotorType.kBrushless){{this.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);}},
                                new CANcoder(33),
                                Inches.of(-14),
                                Inches.of(-14),
                                Degrees.of(-135),
                                pivotControllerBR
                        ),
                        new SwerveModule(//BL
                                new SparkMax(19, SparkLowLevel.MotorType.kBrushless),
                                new SparkMax(18, SparkLowLevel.MotorType.kBrushless){{this.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);}},
                                new CANcoder(34),
                                Inches.of(-14),
                                Inches.of(14),
                                Degrees.of(135),
                                pivotControllerBL
                        ),
                        new Pigeon2(0)

                ));
        this.setElevator(new FakeElevator());

        this.setCoralCollector(new FakeCoralCollector());
    }

}
