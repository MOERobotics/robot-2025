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

        getSensors().swerveModuleFL = swerveModuleFL.inputs;
        getSensors().swerveModuleFR = swerveModuleFR.inputs;
        getSensors().swerveModuleBR = swerveModuleBR.inputs;
        getSensors().swerveModuleBL = swerveModuleBL.inputs;
    }

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
