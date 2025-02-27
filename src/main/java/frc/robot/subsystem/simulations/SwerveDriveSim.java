package frc.robot.subsystem.simulations;


import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystem.SwerveDrive;
import org.littletonrobotics.junction.Logger;
public class SwerveDriveSim {
    private final SwerveDrive swerveDrive;
    private final Pigeon2SimState pigeon2Sim;
    private SwerveModulePosition[] oldModulePositions, currentModulePositions;

    public SwerveDriveSim(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        this.pigeon2Sim = swerveDrive.pigeon.getSimState();
        this.oldModulePositions = new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };
    }

    public void updateSimState() {
        currentModulePositions = swerveDrive.getSensors().modulePositions.clone();
        Twist2d twist = swerveDrive.getKinematics().toTwist2d(oldModulePositions,currentModulePositions);
        Logger.recordOutput("Twist", twist);
        swerveDrive.odometry.resetPose(swerveDrive.getPose().exp(twist));
        pigeon2Sim.setRawYaw(swerveDrive.getPose().getRotation().getRadians());
        oldModulePositions = swerveDrive.getSensors().modulePositions.clone();
    }
}
