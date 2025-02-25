package frc.robot.container;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import frc.robot.subsystem.SwerveDrive;
import frc.robot.subsystem.SwerveModule;
import frc.robot.subsystem.fakes.FakeCoralHead;
import frc.robot.subsystem.fakes.FakeElevator;
import frc.robot.utils.FeedforwardConstants;
import frc.robot.utils.PIDConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
//hello2

public class HortaContainer extends RobotContainer {

    public HortaContainer(){
        double pivotkP = 8.0e-3*60;
        double pivotkI = 0;
        double pivotkD = 0;
        double pivotkIMax = 2;
        double drivekP = 1.0e-3;
        double drivekI = 0.0;
        double drivekD = 1.0e-4;

        double drivekS = 0.19959;
        double drivekV = 0.1233;
        double drivekA = 0.019658;


        PIDConstants pivotFeedback = new PIDConstants(pivotkP, pivotkI, pivotkD, pivotkIMax);
        PIDConstants driveFeedback = new PIDConstants(drivekP, drivekI, drivekD);
        FeedforwardConstants driveFeedForward = new FeedforwardConstants(drivekS, drivekV, drivekA);

        this.setSwerveDrive(
                new SwerveDrive(
                        new SwerveModule(//FL
                                new SparkMax(11, SparkLowLevel.MotorType.kBrushless),
                                new SparkMax(10, SparkLowLevel.MotorType.kBrushless),
                                false,
                                true,
                                new CANcoder(31),
                                Inches.of(14),
                                Inches.of(14),
                                Degrees.of(45),
                                pivotFeedback,
                                driveFeedback,
                                driveFeedForward
                        ),
                        new SwerveModule(//FR
                                new SparkMax(9, SparkLowLevel.MotorType.kBrushless),
                                new SparkMax(8, SparkLowLevel.MotorType.kBrushless),
                                false,
                                true,
                                new CANcoder(32),
                                Inches.of(14),
                                Inches.of(-14),
                                Degrees.of(-45),
                                pivotFeedback,
                                driveFeedback,
                                driveFeedForward
                        ),
                        new SwerveModule(//BR
                                new SparkMax( 1, SparkLowLevel.MotorType.kBrushless),
                                new SparkMax(20, SparkLowLevel.MotorType.kBrushless),
                                false,
                                true,
                                new CANcoder(33),
                                Inches.of(-14),
                                Inches.of(-14),
                                Degrees.of(-135),
                                pivotFeedback,
                                driveFeedback,
                                driveFeedForward
                        ),
                        new SwerveModule(//BL
                                new SparkMax(19, SparkLowLevel.MotorType.kBrushless),
                                new SparkMax(18, SparkLowLevel.MotorType.kBrushless),
                                false,
                                true,
                                new CANcoder(34),
                                Inches.of(-14),
                                Inches.of(14),
                                Degrees.of(135),
                                pivotFeedback,
                                driveFeedback,
                                driveFeedForward
                        ),
                        new Pigeon2(0)

                ));
        this.setElevator(new FakeElevator());

        this.setCoralHead(new FakeCoralHead());
    }

}
