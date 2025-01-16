package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import frc.robot.MOESubsystem;
import org.littletonrobotics.junction.AutoLog;

public class SwerveDrive extends MOESubsystem<SwerveDriveInputsAutoLogged> {
    public SwerveModule swerveModuleFL;
    public SwerveModule swerveModuleFR;
    public SwerveModule swerveModuleBL;
    public SwerveModule swerveModuleBR;
    public SwerveDriveKinematics kinematics;
    public SwerveDriveOdometry odometry;
    public Pigeon2 pigeon;

    @AutoLog
    public static class SwerveDriveInputs {
        double currentRotationDegrees;
        Pose2d pose;
    }

    public SwerveDrive(
            SwerveModule SwerveModuleFL,
            SwerveModule SwerveModuleFR,
            SwerveModule SwerveModuleBR,
            SwerveModule SwerveModuleBL,
            Pigeon2 pigeon) {
        this.swerveModuleFR = SwerveModuleFL;
        this.swerveModuleFL = SwerveModuleFR;
        this.swerveModuleBR = SwerveModuleBR;
        this.swerveModuleBL = SwerveModuleBL;

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
                        this.swerveModuleBL.getModulePosition(),
                        this.swerveModuleBR.getModulePosition(),
                }
        );
    }
    @Override
    public void readSensors(SwerveDriveInputsAutoLogged inputs) {
       inputs.currentRotationDegrees = this.pigeon.getRotation2d().getDegrees();
        inputs.pose = this.odometry.update(
                Rotation2d.kZero,
                new SwerveModulePosition[]{
                        this.swerveModuleFL.getModulePosition(),
                        this.swerveModuleFR.getModulePosition(),
                        this.swerveModuleBR.getModulePosition(),
                        this.swerveModuleBL.getModulePosition()
                }
        );
    }

    public Pose2d getPose() {
        return this.odometry.getPoseMeters();


    }

    public void resetPose(Pose2d newPose) {
        this.odometry.resetPose(newPose);

}
    public void drive(ChassisSpeeds speeds) {

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        swerveModuleFL.drive(moduleStates[0].speedMetersPerSecond);
        swerveModuleFL.pivot(moduleStates[0].angle.getMeasure());

        swerveModuleFR.drive(moduleStates[1].speedMetersPerSecond);
        swerveModuleFR.pivot(moduleStates[1].angle.getMeasure());

        swerveModuleBR.drive(moduleStates[2].speedMetersPerSecond);
        swerveModuleBR.pivot(moduleStates[2].angle.getMeasure());

        swerveModuleBL.drive(moduleStates[3].speedMetersPerSecond);
        swerveModuleBL.pivot(moduleStates[3].angle.getMeasure());
    }

    public void drive(double xSpeed, double ySpeed, double rotation) {
        drive(new ChassisSpeeds(xSpeed, ySpeed, rotation));
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(
                swerveModuleFL.getModuleState(),
                swerveModuleFR.getModuleState(),
                swerveModuleBR.getModuleState(),
                swerveModuleBL.getModuleState()
        );
        return speeds;

    }

}
