package frc.robot.container;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

public class OperatorInterface{
    public static class JoystickAxis implements DoubleSupplier {
        private final CommandJoystick joystick;
        private final int axis;
        private final boolean invert;

        public JoystickAxis(CommandJoystick joystick, int axis) {
            this(joystick, axis, false);
        }
        public JoystickAxis(CommandJoystick joystick, int axis, boolean invert) {
            this.joystick = joystick;
            this.axis = axis;
            this.invert = invert;
        }

        public Trigger greaterThan(double threshold) {
            if (invert)
                return joystick.axisLessThan(axis, -threshold);
            else
                return joystick.axisGreaterThan(axis, threshold);
        }

        public Trigger lessThan(double threshold) {
            if (invert)
                return joystick.axisGreaterThan(axis, -threshold);
            else
                return joystick.axisLessThan(axis, threshold);
        }

        public Trigger absGreaterThan(double threshold) {
            return new Trigger(() -> Math.abs(joystick.getRawAxis(this.axis)) > threshold);
        }

        @Override
        public double getAsDouble() {
            var raw = this.joystick.getRawAxis(this.axis);
            if (invert)
                raw = -raw;
            return raw;
        }

    }
    CommandJoystick driverJoystick = new CommandJoystick(0);
    CommandJoystick funcitonJoystick = new CommandJoystick(1);

    public JoystickAxis getTeleopAxisX() {
        return new JoystickAxis(driverJoystick, 1, true);
    }

    public JoystickAxis getTeleopAxisY() {
        return new JoystickAxis(driverJoystick, 0, true);
    }

    public JoystickAxis getTeleopAxisTheta() {
        return new JoystickAxis(driverJoystick, 4, false);
    }
    public JoystickAxis ElevatorMoveVertically(){
        return new JoystickAxis(driverJoystick, 2, false);
    }
    public JoystickAxis ElevatorMoveHorizontally(){
        return new JoystickAxis(driverJoystick, 3, false);
    }
}
