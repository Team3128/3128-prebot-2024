package frc.team3128.subsystems;

import common.core.subsystems.ManipulatorTemplate;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.wpilibj.DigitalInput;

import static frc.team3128.Constants.HopperConstants.*;

public class Hopper extends ManipulatorTemplate {

    private DigitalInput sensor;

    private static Hopper instance;

    public static synchronized Hopper getInstance() {
        if (instance == null) 
            instance = new Hopper();
        return instance;
    }

    private Hopper() {
        super(STALL_CURRENT, HOPPER_INTAKE_POWER, HOPPER_OUTTAKE_POWER, STALL_POWER, 0.3, HOPPER_MOTOR);
        sensor = new DigitalInput(HOPPER_MOTOR_ID);
        configMotors();
        initShuffleboard();
    }

    @Override
    protected void configMotors() {
        HOPPER_MOTOR.enableVoltageCompensation(VOLT_COMP);
        HOPPER_MOTOR.setNeutralMode(Neutral.BRAKE);
        HOPPER_MOTOR.setCurrentLimit(CURRENT_LIMIT);
        HOPPER_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
        HOPPER_MOTOR.setNeutralMode(Neutral.BRAKE);
    }

    @Override
    public boolean hasObjectPresent() {
        return !sensor.get();
    }
}
