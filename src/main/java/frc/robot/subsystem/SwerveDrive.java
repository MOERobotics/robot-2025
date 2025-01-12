package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    public SwerveModule SwervemoduleFL;
    public SwerveModule SwervemoduleFR;
    public SwerveModule SwervemoduleBL;
    public SwerveModule SwervemoduleBR;
    public SwerveDriveKinematics kinematics;
    public SwerveDriveOdometry odometry;

    public static class Inputs {
        double initialRotation;
        double currentRotation;
    }

    public SwerveDrive(
            SwerveModule SwerveModuleFL,
            SwerveModule SwerveModuleFR,
            SwerveModule SwerveModuleBR,
            SwerveModule SwerveModuleBL) {
        this.SwervemoduleBR = SwerveModuleBR;
        this.SwervemoduleBL = SwerveModuleBL;
        this.SwervemoduleFR = SwerveModuleFR;
        this.SwervemoduleFL = SwerveModuleFL;
        kinematics = new SwerveDriveKinematics(
                new Translation2d(SwerveModuleFL.xpos, SwerveModuleFL.ypos),
                new Translation2d(SwerveModuleFR.xpos, SwerveModuleFR.ypos),
                new Translation2d(SwerveModuleBR.xpos, SwerveModuleBR.ypos),
                new Translation2d(SwerveModuleBL.xpos, SwerveModuleBL.ypos)
        );
        this.odometry = new SwerveDriveOdometry(
                this.kinematics,
                Rotation2d.kZero,
                new SwerveModulePosition[]{
                        this.SwervemoduleFL.getModulePosition(),
                        this.SwervemoduleFR.getModulePosition(),
                        this.SwervemoduleBL.getModulePosition(),
                        this.SwervemoduleBR.getModulePosition(),
                }
        );
    }

    public void updateSensors() {
        this.odometry.update(
                Rotation2d.kZero,
                new SwerveModulePosition[]{
                        this.SwervemoduleFL.getModulePosition(),
                        this.SwervemoduleFR.getModulePosition(),
                        this.SwervemoduleBL.getModulePosition(),
                        this.SwervemoduleBR.getModulePosition()
                }
        );
    }

    public Pose2d getPose() {
        this.odometry.getPoseMeters();
    }

    public void resetPose(Pose2d NewPose) {

            this.odometry.getPoseMeters();
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
