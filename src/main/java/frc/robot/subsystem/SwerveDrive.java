package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
    public SwerveDrivePoseEstimator poseEstimator;
    public Pigeon2 pigeon;
    public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    public SysIdRoutine[] pivotSysIdRoutines;

    public SysIdRoutine sysIdRoutinePivotFL = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.4).per(Second), Volts.of(3.5), Seconds.of(5.0), (state) -> Logger.recordOutput("SysIDTestStateFL", state.toString())
            ),
            new SysIdRoutine.Mechanism((voltage) -> {
                double power = voltage.in(Volts);
                swerveModuleFL.pivotVolts(power);
            }, null, this));

    public SysIdRoutine sysIdRoutinePivotFR = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.4).per(Second), Volts.of(3.5), Seconds.of(5.0), (state) -> Logger.recordOutput("SysIDTestStateFR", state.toString())
            ),
            new SysIdRoutine.Mechanism((voltage) -> {
                double power = voltage.in(Volts);
                swerveModuleFR.pivotVolts(power);
            }, null, this));

    public SysIdRoutine sysIdRoutinePivotBL = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.4).per(Second), Volts.of(3.5), Seconds.of(5.0), (state) -> Logger.recordOutput("SysIDTestStateBL", state.toString())
            ),
            new SysIdRoutine.Mechanism((voltage) -> {
                double power = voltage.in(Volts);
                swerveModuleBL.pivotVolts(power);
            }, null, this));

    public SysIdRoutine sysIdRoutinePivotBR = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.4).per(Second), Volts.of(3.5), Seconds.of(5.0), (state) -> Logger.recordOutput("SysIDTestStateBR", state.toString())
            ),
            new SysIdRoutine.Mechanism((voltage) -> {
                double power = voltage.in(Volts);
                swerveModuleBR.pivotVolts(power);
            }, null, this));
    public SysIdRoutine sysIdRoutineDrive = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.3).per(Second),
                    Volts.of(1.5),
                    Seconds.of(5.0),
                    (state) -> Logger.recordOutput("SysIDTestStateDrive", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                    double power = voltage.in(Volts);
                    swerveModuleFL.pivot(Degrees.of(0));
                    swerveModuleFL.drive(power);
                    swerveModuleBR.pivot(Degrees.of(0));
                    swerveModuleBR.drive(power);
                    swerveModuleBL.pivot(Degrees.of(0));
                    swerveModuleBL.drive(power);
                    swerveModuleFR.pivot(Degrees.of(0));
                    swerveModuleFR.drive(power);
                },
                null,
                this
            )
    );

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
        this.swerveModules = new SwerveModule[]{swerveModuleFL,swerveModuleFR,swerveModuleBR,swerveModuleBL};
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

        this.poseEstimator = new SwerveDrivePoseEstimator(
                this.kinematics,
                this.pigeon.getRotation2d(),
                new SwerveModulePosition[]{
                        this.swerveModuleFL.getModulePosition(),
                        this.swerveModuleFR.getModulePosition(),
                        this.swerveModuleBR.getModulePosition(),
                        this.swerveModuleBL.getModulePosition(),
                },
                new Pose2d()
        );

        getSensors().moduleStates = new SwerveModuleState[4];
        getSensors().modulePositions = new SwerveModulePosition[4];

        getSensors().swerveModuleFL = swerveModuleFL.getSensors();
        getSensors().swerveModuleFR = swerveModuleFR.getSensors();
        getSensors().swerveModuleBR = swerveModuleBR.getSensors();
        getSensors().swerveModuleBL = swerveModuleBL.getSensors();
    }
    // public Pose2d visionMeasurement(){
    // Pose2d robotVisionPosition = swerveDrivePoseEstimator.addVisionMeasurement(null);
    // }
//    public Pose2d calculatePosition(){
//        Pose2d calcPosition = swerveDrivePoseEstimator.getEstimatedPosition();
//        return calcPosition;
//    }

    @Override
    public void readSensors(SwerveDriveInputsAutoLogged sensors) {
        sensors.currentRotationRadians = this.pigeon.getRotation2d().getRadians();
        sensors.pose = this.odometry.update(
            this.pigeon.getRotation2d(),
            new SwerveModulePosition[]{
                this.swerveModuleFL.getModulePosition(),
                this.swerveModuleFR.getModulePosition(),
                this.swerveModuleBR.getModulePosition(),
                this.swerveModuleBL.getModulePosition()
            }
        );
        this.poseEstimator.update(
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



        sensors.moduleStates[0] = swerveModuleFL.getModuleState();
        sensors.moduleStates[1] = swerveModuleFR.getModuleState();
        sensors.moduleStates[2] = swerveModuleBR.getModuleState();
        sensors.moduleStates[3] = swerveModuleBL.getModuleState();

        sensors.modulePositions[0] = swerveModuleFL.getModulePosition();
        sensors.modulePositions[1] = swerveModuleFR.getModulePosition();
        sensors.modulePositions[2] = swerveModuleBR.getModulePosition();
        sensors.modulePositions[3] = swerveModuleBL.getModulePosition();

        sensors.robotRelativeSpeeds = kinematics.toChassisSpeeds(
                swerveModuleFL.getModuleState(),
                swerveModuleFR.getModuleState(),
                swerveModuleBR.getModuleState(),
                swerveModuleBL.getModuleState());

    }
    /*
    // pose 2d, float seconds,
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {

    }

    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
            setVisionMeasurementStdDevs(visionMeasurementStdDevs);
            addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
          }
      */

    @Override
    public void drive(ChassisSpeeds speeds) {
        Logger.recordOutput("ChassisSpeeds", speeds);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        this.getSensors().driveDesiredStates = moduleStates;
        swerveModuleFL.setModuleState(moduleStates[0]);
        swerveModuleFR.setModuleState(moduleStates[1]);
        swerveModuleBR.setModuleState(moduleStates[2]);
        swerveModuleBL.setModuleState(moduleStates[3]);

        /*swerveModuleFL.drive(moduleStates[0].speedMetersPerSecond);
        swerveModuleFL.pivot(moduleStates[0].angle.getMeasure());

        swerveModuleFR.drive(moduleStates[1].speedMetersPerSecond);
        swerveModuleFR.pivot(moduleStates[1].angle.getMeasure());

        swerveModuleBR.drive(moduleStates[2].speedMetersPerSecond);
        swerveModuleBR.pivot(moduleStates[2].angle.getMeasure());

        swerveModuleBL.drive(moduleStates[3].speedMetersPerSecond);
        swerveModuleBL.pivot(moduleStates[3].angle.getMeasure());*/

    }

    /**
     * Drives an individual swerve module of the swerve drive
     *
     * @param index    the index of the swerve module 0-3 represent modules FL,FR,BR,BL
     */
    @Override
    public void driveSingleModule(int index, double xSpeed, double ySpeed, double rotation) {
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed,ySpeed,rotation);
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        this.getSensors().driveDesiredStates = moduleStates;
        swerveModules[index].drive(moduleStates[index].speedMetersPerSecond);
        swerveModules[index].pivot(moduleStates[index].angle.getMeasure());
    }

    @Override
    public Command sysIDCommandsPivot(CommandType commandType, ModuleType moduleType) {
        SysIdRoutine currentRoutine = null;
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
        switch (commandType) {
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
        return Commands.none();
    }
    public void pivotAngle(Angle angle){
        for(SwerveModule i:swerveModules){
            i.pivot(angle);
        }
    }


}
