package frc.team3128.subsystems;

import common.core.subsystems.ManipulatorTemplate;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.wpilibj.DigitalInput;
import common.utility.shuffleboard.NAR_Shuffleboard;


import static frc.team3128.Constants.HopperConstants.*;

public class Hopper extends ManipulatorTemplate {

    // front is the shooter
    private DigitalInput frontSensor, backSensor;

    private static Hopper instance;

    public static synchronized Hopper getInstance() {
        if (instance == null) 
            instance = new Hopper();
        return instance;
    }

    private Hopper() {
        super(STALL_CURRENT, HOPPER_INTAKE_POWER, HOPPER_OUTTAKE_POWER, STALL_POWER, 0.3, HOPPER_MOTOR);
        frontSensor = new DigitalInput(HOPPER_FRONT_SENSOR_ID);
        // backSensor = new DigitalInput(HOPPER_BACK_SENSOR_ID);
        configMotors();
        NAR_Shuffleboard.addData(getName(), "Has Object Present", ()-> hasObjectPresent(), 0, 0);
        // initShuffleboard();
    }

    @Override
    protected void configMotors() {
        HOPPER_MOTOR.enableVoltageCompensation(VOLT_COMP);
        HOPPER_MOTOR.setCurrentLimit(CURRENT_LIMIT);

        HOPPER_MOTOR.setNeutralMode(Neutral.COAST);

        HOPPER_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
    }

    @Override
    public boolean hasObjectPresent() {
        return noteInFront();
        // return noteInFront() || noteInBack();
    }

    // shooter side
    public boolean noteInFront() {
        return !frontSensor.get();
    }
}
