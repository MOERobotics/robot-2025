package frc.robot.subsystem.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystem.interfaces.SwerveDriveInputsAutoLogged;
import frc.robot.subsystem.interfaces.SwerveModuleInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveDriveControl extends Subsystem {
    default Pose2d getPose() {
        return this.getSensors().pose;
    }

    SwerveDriveInputsAutoLogged getSensors();

    default void resetPose(Pose2d newPose) {
        this.getOdometry().resetPose(newPose);
    }

    void drive(ChassisSpeeds speeds);

    default void drive(double xSpeed, double ySpeed, double rotation) {
        drive(new ChassisSpeeds(xSpeed, ySpeed, rotation));
    }

    void driveSingleModule (int index, double xSpeed, double ySpeed, double rotation);

    default ChassisSpeeds getRobotRelativeSpeeds() {
        return this.getSensors().robotRelativeSpeeds;
    }

    edu.wpi.first.math.kinematics.SwerveDriveKinematics getKinematics();




        edu.wpi.first.math.kinematics.SwerveDriveOdometry getOdometry();

    @AutoLog
    public static class SwerveDriveInputs {
        public double currentRotationRadians;
        public Pose2d pose;
        public SwerveModuleState[] moduleStates;
        public SwerveModuleState[] driveDesiredStates;
        public SwerveModulePosition[] modulePositions;
        public SwerveModuleInputsAutoLogged swerveModuleFL;
        public SwerveModuleInputsAutoLogged swerveModuleFR;
        public SwerveModuleInputsAutoLogged swerveModuleBR;
        public SwerveModuleInputsAutoLogged swerveModuleBL;
        public ChassisSpeeds robotRelativeSpeeds;
    }
}
