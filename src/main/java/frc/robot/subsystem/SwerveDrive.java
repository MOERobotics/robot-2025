package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import frc.robot.MOESubsystem;
import lombok.Getter;

public class SwerveDrive extends MOESubsystem<SwerveDriveInputsAutoLogged> implements SwerveDriveControl {
    public SwerveModule swerveModuleFL;
    public SwerveModule swerveModuleFR;
    public SwerveModule swerveModuleBL;
    public SwerveModule swerveModuleBR;
    public @Getter SwerveDriveKinematics kinematics;
    public @Getter SwerveDriveOdometry odometry;
    public Pigeon2 pigeon;


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

}
