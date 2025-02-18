package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import frc.robot.MOESubsystem;
import frc.robot.subsystem.interfaces.SwerveDriveControl;
import frc.robot.subsystem.interfaces.SwerveDriveInputsAutoLogged;
import lombok.Getter;

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

    public SwerveDrive(
        SwerveModule SwerveModuleFL,
        SwerveModule SwerveModuleFR,
        SwerveModule SwerveModuleBR,
        SwerveModule SwerveModuleBL,
        Pigeon2 pigeon
    ) {

        super(new SwerveDriveInputsAutoLogged());
        this.swerveModuleBR = SwerveModuleBR;
        this.swerveModuleBL = SwerveModuleBL;
        this.swerveModuleFR = SwerveModuleFR;
        this.swerveModuleFL = SwerveModuleFL;

        SparkMaxConfig FLConfig = new SparkMaxConfig();
        SparkMaxConfig FRConfig = new SparkMaxConfig();
        SparkMaxConfig BRConfig = new SparkMaxConfig();
        SparkMaxConfig BLConfig = new SparkMaxConfig();

        FLConfig.inverted(true);
        FRConfig.inverted(true);
        BRConfig.inverted(true);
        BLConfig.inverted(true);

        SwerveModuleFL.driveMotor.configure(FLConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        SwerveModuleFR.driveMotor.configure(FRConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        SwerveModuleBR.driveMotor.configure(BRConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        SwerveModuleBL.driveMotor.configure(BLConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        this.swerveModules = new SwerveModule[]{swerveModuleFL,SwerveModuleFR,swerveModuleBR,swerveModuleBL};
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
                swerveModuleBL.getModuleState()
        );


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
    public void drive(double xSpeed, double ySpeed, double rotation) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed,rotation,getPose().getRotation()));
    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        this.getSensors().driveDesiredStates = moduleStates;
        /*
        swerveModuleFL.setModuleState(moduleStates[0]);
        swerveModuleFR.setModuleState(moduleStates[1]);
        swerveModuleBR.setModuleState(moduleStates[2]);
        swerveModuleBL.setModuleState(moduleStates[3]);



         */


        swerveModuleFL.setModuleState(moduleStates[0]);

        swerveModuleFR.setModuleState(moduleStates[1]);

        swerveModuleBR.setModuleState(moduleStates[2]);

        swerveModuleBL.setModuleState(moduleStates[3]);



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

}
