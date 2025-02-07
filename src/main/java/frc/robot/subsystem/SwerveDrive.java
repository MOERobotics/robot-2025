package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.MOESubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class SwerveDrive extends MOESubsystem<SwerveDriveInputsAutoLogged> implements SwerveDriveControl {
    public SwerveModule swerveModuleFL;
    public SwerveModule swerveModuleFR;
    public SwerveModule swerveModuleBL;
    public SwerveModule swerveModuleBR;
    public SwerveModule[] swerveModules;
    public @Getter SwerveDriveKinematics kinematics;
    public @Getter SwerveDriveOdometry odometry;
    public Pigeon2 pigeon;
    public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    public SysIdRoutine[] pivotSysIdRoutines;

    public SysIdRoutine sysIdRoutinePivotFL = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.2).per(Second),Volts.of(3.5),Seconds.of(5.0), (state) -> Logger.recordOutput("SysIDTestStateFL", state.toString())
            ),
            new SysIdRoutine.Mechanism((voltage) -> {
                double power = voltage.in(Volts);
                swerveModuleFL.pivotVolts(power);
            }, null, this));

    public SysIdRoutine sysIdRoutinePivotFR = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.2).per(Second),Volts.of(3.5),Seconds.of(5.0), (state) -> Logger.recordOutput("SysIDTestStateFR", state.toString())
            ),
            new SysIdRoutine.Mechanism((voltage) -> {
                double power = voltage.in(Volts);
                swerveModuleFR.pivotVolts(power);
            }, null, this));

    public SysIdRoutine sysIdRoutinePivotBL = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.2).per(Second),Volts.of(3.5),Seconds.of(5.0), (state) -> Logger.recordOutput("SysIDTestStateBL", state.toString())
            ),
            new SysIdRoutine.Mechanism((voltage) -> {
                double power = voltage.in(Volts);
                swerveModuleBL.pivotVolts(power);
            }, null, this));

    public SysIdRoutine sysIdRoutinePivotBR = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.2).per(Second),Volts.of(3.5),Seconds.of(5.0), (state) -> Logger.recordOutput("SysIDTestStateBR", state.toString())
            ),
            new SysIdRoutine.Mechanism((voltage) -> {
                double power = voltage.in(Volts);
                swerveModuleBR.pivotVolts(power);
            }, null, this));
    public SysIdRoutine sysIdRoutineDrive = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.2).per(Second),Volts.of(3.5),Seconds.of(5.0), (state) -> Logger.recordOutput("SysIDTestStateDrive", state.toString())
            ),
            new SysIdRoutine.Mechanism((voltage) -> {
                double power = voltage.in(Volts);
                swerveModuleFL.drive(power);
                swerveModuleBR.drive(power);
                swerveModuleBL.drive(power);
                swerveModuleFR.drive(power);
            }, null, this));





    public SwerveDrive(
        SwerveModule SwerveModuleFL,
        SwerveModule SwerveModuleFR,
        SwerveModule SwerveModuleBR,
        SwerveModule SwerveModuleBL,
        Pigeon2 pigeon
    ) {
        this.setSensors(new SwerveDriveInputsAutoLogged());
        this.swerveModuleBR = SwerveModuleBR;
        this.swerveModuleBL = SwerveModuleBL;
        this.swerveModuleFR = SwerveModuleFR;
        this.swerveModuleFL = SwerveModuleFL;
        this.pigeon = pigeon;
        kinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveModuleFL.xPos, SwerveModuleFL.yPos),
            new Translation2d(SwerveModuleFR.xPos, SwerveModuleFR.yPos),
            new Translation2d(SwerveModuleBR.xPos, SwerveModuleBR.yPos),
            new Translation2d(SwerveModuleBL.xPos, SwerveModuleBL.yPos)
        );
        this.odometry = new SwerveDriveOdometry(
            this.kinematics,
            this.pigeon.getRotation2d(),
            new SwerveModulePosition[]{
                this.swerveModuleFL.getModulePosition(),
                this.swerveModuleFR.getModulePosition(),
                this.swerveModuleBR.getModulePosition(),
                this.swerveModuleBL.getModulePosition(),
            }
        );
        getSensors().moduleStates = new SwerveModuleState[4];
        getSensors().modulePositions = new SwerveModulePosition[4];

        getSensors().swerveModuleFL = swerveModuleFL.inputs;
        getSensors().swerveModuleFR = swerveModuleFR.inputs;
        getSensors().swerveModuleBR = swerveModuleBR.inputs;
        getSensors().swerveModuleBL = swerveModuleBL.inputs;
    }
    // public Pose2d visionMeasurement(){
       // Pose2d robotVisionPosition = swerveDrivePoseEstimator.addVisionMeasurement(null);
    // }
//    public Pose2d calculatePosition(){
//        Pose2d calcPosition = swerveDrivePoseEstimator.getEstimatedPosition();
//        return calcPosition;
//    }

    @Override
    public void readSensors(SwerveDriveInputsAutoLogged inputs) {


        inputs.currentRotationRadians = this.pigeon.getRotation2d().getRadians();
        inputs.pose = this.odometry.update(
            this.pigeon.getRotation2d(),
            new SwerveModulePosition[]{
                this.swerveModuleFL.getModulePosition(),
                this.swerveModuleFR.getModulePosition(),
                this.swerveModuleBR.getModulePosition(),
                this.swerveModuleBL.getModulePosition()
            }
        );
//        this.swerveDrivePoseEstimator.update(
//                this.pigeon.getRotation2d(),
//                new SwerveModulePosition[]{
//                        this.swerveModuleFL.getModulePosition(),
//                        this.swerveModuleFR.getModulePosition(),
//                        this.swerveModuleBR.getModulePosition(),
//                        this.swerveModuleBL.getModulePosition()
//                }
//        );

        inputs.moduleStates[0] = swerveModuleFL.getModuleState();
        inputs.moduleStates[1] = swerveModuleFR.getModuleState();
        inputs.moduleStates[2] = swerveModuleBR.getModuleState();
        inputs.moduleStates[3] = swerveModuleBL.getModuleState();

        inputs.modulePositions[0] = swerveModuleFL.getModulePosition();
        inputs.modulePositions[1] = swerveModuleFR.getModulePosition();
        inputs.modulePositions[2] = swerveModuleBR.getModulePosition();
        inputs.modulePositions[3] = swerveModuleBL.getModulePosition();

        swerveModuleFL.readSensors();
        swerveModuleFR.readSensors();
        swerveModuleBR.readSensors();
        swerveModuleBL.readSensors();
        inputs.robotRelativeSpeeds = kinematics.toChassisSpeeds(
                swerveModuleFL.getModuleState(),
                swerveModuleFR.getModuleState(),
                swerveModuleBR.getModuleState(),
                swerveModuleBL.getModuleState());

    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        this.getSensors().driveDesiredStates = moduleStates;
        swerveModuleFL.drive(moduleStates[0].speedMetersPerSecond);
        swerveModuleFL.pivot(moduleStates[0].angle.getMeasure());

        swerveModuleFR.drive(moduleStates[1].speedMetersPerSecond);
        swerveModuleFR.pivot(moduleStates[1].angle.getMeasure());

        swerveModuleBR.drive(moduleStates[2].speedMetersPerSecond);
        swerveModuleBR.pivot(moduleStates[2].angle.getMeasure());

        swerveModuleBL.drive(moduleStates[3].speedMetersPerSecond);
        swerveModuleBL.pivot(moduleStates[3].angle.getMeasure());

    }

    @Override
    public Command sysIDCommands(CommandType commandType, ModuleType moduleType, DriveOrPivot driveOrPivot) {
        SysIdRoutine currentRoutine = null;
        String type = " ";
        if (type.equals("pivot")){
        switch (moduleType) {
            case modFL -> {
                currentRoutine = sysIdRoutinePivotFL;
            }
            case modFR -> {
                currentRoutine = sysIdRoutinePivotFR;
            }
            case modBL -> {
                currentRoutine = sysIdRoutinePivotBL;
            }
            case modBR -> {
                currentRoutine = sysIdRoutinePivotBR;
            }
        }


        }
       switch(commandType){
           case QuasistaticForward -> {
               return currentRoutine.quasistatic(SysIdRoutine.Direction.kForward);
           }
           case QuasistaticReverse -> {
               return currentRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
           }
           case DynamicForward -> {
               return currentRoutine.dynamic(SysIdRoutine.Direction.kForward);
           }
           case DynamicReverse -> {
               return currentRoutine.dynamic(SysIdRoutine.Direction.kReverse);
           }
 
        }
        switch (driveOrPivot){
            case setPivot -> {
                type = "pivot";
            }
            case setDrive -> {
                currentRoutine = sysIdRoutineDrive;
            }
        }



        return Commands.none();
    }


}
