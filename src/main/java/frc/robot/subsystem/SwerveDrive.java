package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MOESubsystem;
import org.littletonrobotics.junction.AutoLog;

public class SwerveDrive extends MOESubsystem<InputsAutoLogged> {
    public SwerveModule SwervemoduleFL;
    public SwerveModule SwervemoduleFR;
    public SwerveModule SwervemoduleBL;
    public SwerveModule SwervemoduleBR;
    public SwerveDriveKinematics kinematics;
    public SwerveDriveOdometry odometry;
    public Pigeon2 pigeon;

    @AutoLog
    public static class Inputs {
        double currentRotation;
        Pose2d pose;
    }

    public SwerveDrive(
        SwerveModule SwerveModuleFL,
        SwerveModule SwerveModuleFR,
        SwerveModule SwerveModuleBR,
        SwerveModule SwerveModuleBL,
        Pigeon2 pigeon
    ) {
        this.SwervemoduleBR = SwerveModuleBR;
        this.SwervemoduleBL = SwerveModuleBL;
        this.SwervemoduleFR = SwerveModuleFR;
        this.SwervemoduleFL = SwerveModuleFL;
        this.pigeon = pigeon;
        kinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveModuleFL.xpos, SwerveModuleFL.ypos),
            new Translation2d(SwerveModuleFR.xpos, SwerveModuleFR.ypos),
            new Translation2d(SwerveModuleBR.xpos, SwerveModuleBR.ypos),
            new Translation2d(SwerveModuleBL.xpos, SwerveModuleBL.ypos)
        );
        this.odometry = new SwerveDriveOdometry(
            this.kinematics,
            this.pigeon.getRotation2d(),
            new SwerveModulePosition[]{
                this.SwervemoduleFL.getModulePosition(),
                this.SwervemoduleFR.getModulePosition(),
                this.SwervemoduleBL.getModulePosition(),
                this.SwervemoduleBR.getModulePosition(),
            }
        );
    }

    @Override
    public void readSensors(InputsAutoLogged inputs) {
        inputs.currentRotation = this.pigeon.getRotation2d().getDegrees();
        inputs.pose = this.odometry.update(
            this.pigeon.getRotation2d(),
            new SwerveModulePosition[]{
                this.SwervemoduleFL.getModulePosition(),
                this.SwervemoduleFR.getModulePosition(),
                this.SwervemoduleBL.getModulePosition(),
                this.SwervemoduleBR.getModulePosition()
            }
        );
    }

    public Pose2d getPose() {
        return this.odometry.getPoseMeters();
    }

    public void resetPose(Pose2d NewPose) {
        this.odometry.resetPose(NewPose);
    }

    public void Drive(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        SwervemoduleFL.drive(moduleStates[0].speedMetersPerSecond);
        SwervemoduleFL.pivot(moduleStates[0].angle.getMeasure());
        SwervemoduleFL.drive(moduleStates[1].speedMetersPerSecond);
        SwervemoduleFR.pivot(moduleStates[1].angle.getMeasure());
        SwervemoduleBR.drive(moduleStates[2].speedMetersPerSecond);
        SwervemoduleBR.pivot(moduleStates[2].angle.getMeasure());
        SwervemoduleBL.drive(moduleStates[3].speedMetersPerSecond);
        SwervemoduleBL.pivot(moduleStates[3].angle.getMeasure());
    }

    public void Drive(double xSpeed, double ySpeed, double rotation) {
        Drive(new ChassisSpeeds(xSpeed, ySpeed, rotation));
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(
            SwervemoduleFL.getmodulestate(),
            SwervemoduleFR.getmodulestate(),
            SwervemoduleBL.getmodulestate(),
            SwervemoduleBR.getmodulestate()
        );
        return speeds;
    }

}
