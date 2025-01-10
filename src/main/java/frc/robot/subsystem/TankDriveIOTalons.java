package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;

public class TankDriveIOTalons implements TankDriveIO{

    private WPI_TalonSRX left1 = new WPI_TalonSRX(1);

    private WPI_TalonSRX left2 = new WPI_TalonSRX(2);
    private WPI_TalonSRX left3 = new WPI_TalonSRX(3);


    private WPI_TalonSRX right1 = new WPI_TalonSRX(12);


    private WPI_TalonSRX right2 = new WPI_TalonSRX(13);

    private WPI_TalonSRX right3 = new WPI_TalonSRX(14);


    private Encoder leftencoder = new Encoder(0, 1, false, CounterBase.EncodingType.k1X);




    public TankDriveIOTalons(){
        //TODO: set up motors, sensors

        left1.setInverted(false);
        left2.setInverted(false);
        left3.setInverted(false);

        right1.setInverted(true);
        right2.setInverted(true);
        right3.setInverted(true);

        leftencoder.reset();




    }
    @Override
    public void updateInputs(TankDriveInputs inputs) {
        //TODO: Update inputs
        inputs.encoderTicksLeft = leftencoder.get();
        inputs.encoderTicksRight = leftencoder.get();

        inputs.motorPosition = leftencoder.get()/112*12*Math.PI;

        inputs.motorVelocity = leftencoder.getRate()/112*12*Math.PI;

        inputs.appliedVoltage = left1.getMotorOutputVoltage();


    }

    @Override
    public void drive(double left, double right) {
        //TODO: drive motors
        left1.set(TalonSRXControlMode.PercentOutput, left);
        left2.set(TalonSRXControlMode.PercentOutput, left);
        left3.set(TalonSRXControlMode.PercentOutput, left);

        right1.set(TalonSRXControlMode.PercentOutput, right);
        right2.set(TalonSRXControlMode.PercentOutput, right);
        right3.set(TalonSRXControlMode.PercentOutput, right);

    }
    @Override
    public void setVoltageLeft(double voltageLeft) {

        left1.setVoltage(voltageLeft);
        left2.setVoltage(voltageLeft);
        left3.setVoltage(voltageLeft);



    }


    @Override
    public void setVoltageRight(double voltageRight) {

        right1.setVoltage(voltageRight);
        right2.setVoltage(voltageRight);
        right3.setVoltage(voltageRight);

    }



}