package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveDriveController extends Subsystem {
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

    default ChassisSpeeds getRobotRelativeSpeeds() {
        return this.getSensors().robotRelativeSpeeds;
    }

    edu.wpi.first.math.kinematics.SwerveDriveKinematics getKinematics();

    edu.wpi.first.math.kinematics.SwerveDriveOdometry getOdometry();

    @AutoLog
    public static class SwerveDriveInputs {
        double currentRotationRadians;
        Pose2d pose;
        SwerveModuleState[] moduleStates;
        SwerveModuleState[] driveDesiredStates;
        SwerveModulePosition[] modulePositions;
        SwerveModuleInputsAutoLogged swerveModuleFL;
        SwerveModuleInputsAutoLogged swerveModuleFR;
        SwerveModuleInputsAutoLogged swerveModuleBR;
        SwerveModuleInputsAutoLogged swerveModuleBL;
        ChassisSpeeds robotRelativeSpeeds;
    }
}
