package frc.robot.subsystem.interfaces;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public interface SwerveDriveControl extends Subsystem {
    @AutoLog
    class SwerveDriveInputs {
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

        public boolean canClimb;
    }

    default Pose2d getPose() {
        return this.getSensors().pose;
    }

    SwerveDriveInputsAutoLogged getSensors();

    default void resetPose(Pose2d newPose) {
        this.getOdometry().resetPose(newPose);
    }

    void drive(ChassisSpeeds speeds);

    default void drive(double xSpeed, double ySpeed, double rotation, boolean robotCentric) {
        drive(new ChassisSpeeds(xSpeed, ySpeed, rotation));
    }

    default void drive(double xSpeed, double ySpeed, double rotation) {
        drive(xSpeed, ySpeed, rotation, false);
    }

    default ChassisSpeeds getRobotRelativeSpeeds() {
        return this.getSensors().robotRelativeSpeeds;
    }

    SwerveDriveKinematics getKinematics();

    SwerveDrivePoseEstimator getOdometry();

    default boolean canClimb() {
        return this.getSensors().canClimb;
    }


}
