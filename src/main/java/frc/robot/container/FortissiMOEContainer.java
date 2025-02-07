package frc.robot.container;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystem.*;

import java.util.Set;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public class FortissiMOEContainer extends RobotContainer {

    SendableChooser<SwerveDriveControl.CommandType> chooser = new SendableChooser<>();
    SendableChooser<SwerveDriveControl.ModuleType> modChooser = new SendableChooser<>();
    SendableChooser<SwerveDriveControl.DriveOrPivot> driveTypeChooser = new SendableChooser<>();
    public FortissiMOEContainer (){
        double pivotkP = 0.01;
        double pivotkI = 0.1;
        double pivotkD = 0;

        PIDController pivotControllerFL = new PIDController(pivotkP, pivotkI, pivotkD);
        //pivotControllerFL.setIntegratorRange(-pivotkIMax, pivotkIMax);
        PIDController pivotControllerFR = new PIDController(pivotkP, pivotkI, pivotkD);
       // pivotControllerFR.setIntegratorRange(-pivotkIMax, pivotkIMax);
        PIDController pivotControllerBR = new PIDController(pivotkP, pivotkI, pivotkD);
       // pivotControllerBR.setIntegratorRange(-pivotkIMax, pivotkIMax);
        PIDController pivotControllerBL = new PIDController(pivotkP, pivotkI, pivotkD);
       // pivotControllerBL.setIntegratorRange(-pivotkIMax, pivotkIMax);
        pivotControllerBR.enableContinuousInput(-Math.PI, Math.PI);
        pivotControllerBL.enableContinuousInput(-Math.PI, Math.PI);
        pivotControllerFR.enableContinuousInput(-Math.PI, Math.PI);
        pivotControllerFL.enableContinuousInput(-Math.PI, Math.PI);

        chooser.addOption("SysTDStatic_Forward",SwerveDriveControl.CommandType.QuasistaticForward);
        chooser.addOption("SysTDStatic_Reverse", SwerveDriveControl.CommandType.QuasistaticReverse);
        chooser.addOption("SysTDDynamic_Forward", SwerveDriveControl.CommandType.DynamicForward);
        chooser.setDefaultOption("SysTDDynamic_Reverse", SwerveDriveControl.CommandType.DynamicReverse);
        SmartDashboard.putData("Command Chooser",chooser);

        modChooser.addOption("All Modules", SwerveDriveControl.ModuleType.allMods);
        modChooser.addOption("FL", SwerveDriveControl.ModuleType.modFL);
        modChooser.addOption("FR", SwerveDriveControl.ModuleType.modFR);
        modChooser.addOption("BL", SwerveDriveControl.ModuleType.modBL);
        modChooser.setDefaultOption("BR", SwerveDriveControl.ModuleType.modBR);
        SmartDashboard.putData("Module Chooser",modChooser);

        driveTypeChooser.setDefaultOption("Pivot", SwerveDriveControl.DriveOrPivot.setPivot);




        // drive.setDefaultCommand(Commands.run(() -> drive.drive(operatorInterface.getTeleopAxisX().getAsDouble(), operatorInterface.getTeleopAxisY().getAsDouble()), drive).withName("Joystick Control"));
        this.setSwerveDrive(
                new SwerveDrive(
                        new SwerveModule(//FL
                                new SparkMax(17, SparkLowLevel.MotorType.kBrushless),
                                new SparkMax(16, SparkLowLevel.MotorType.kBrushless){{this.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);}},
                                new CANcoder(34),
                                Inches.of(14),
                                Inches.of(14),
                                Degrees.of(45),
                                pivotControllerFL
                        ),
                        new SwerveModule(//FR
                                new SparkMax(3, SparkLowLevel.MotorType.kBrushless),
                                new SparkMax(2, SparkLowLevel.MotorType.kBrushless){{this.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);}},
                                new CANcoder(33),
                                Inches.of(14),
                                Inches.of(-14),
                                Degrees.of(-45),
                                pivotControllerFR
                        ),
                        new SwerveModule(//BR
                                new SparkMax( 1, SparkLowLevel.MotorType.kBrushless),
                                new SparkMax(20, SparkLowLevel.MotorType.kBrushless){{this.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);}},
                                new CANcoder(32),
                                Inches.of(-14),
                                Inches.of(-14),
                                Degrees.of(-135),
                                pivotControllerBR
                        ),
                        new SwerveModule(//BL
                                new SparkMax(19, SparkLowLevel.MotorType.kBrushless),
                                new SparkMax(18, SparkLowLevel.MotorType.kBrushless){{this.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);}},
                                new CANcoder(31),
                                Inches.of(-14),
                                Inches.of(14),
                                Degrees.of(135),
                                pivotControllerBL
                        ),
                        new Pigeon2(0)

        ));
        this.setElevator(new FakeElevator());

        this.setCoralCollector(new FakeCoralCollector());

        var sysidCommand = Commands.defer(() -> {
            return getSwerveDrive().sysIDCommands(chooser.getSelected(), modChooser.getSelected(), driveTypeChooser.getSelected());
        }, Set.of(getSwerveDrive()));
        SmartDashboard.putData("SysIDCommand", sysidCommand);
    }
}


