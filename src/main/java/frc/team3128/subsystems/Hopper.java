package frc.team3128.subsystems;

import common.core.subsystems.ManipulatorTemplate;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

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

    // public Command runHopper(double power){
    //     return runOnce(() -> HOPPER_MOTOR.set(power));
    // }

    // public Command intakeHopper(){
    //     return runHopper(HOPPER_INTAKE_POWER);
    // }

    // public Command outtakeHopper(){
    //     return runHopper(HOPPER_OUTTAKE_POWER);
    // }

    // public Command stopHopper(){
    //     return runHopper(0);
    // }

    // // intake side
    // public boolean noteInBack() {
    //     return !backSensor.get();
    // }
}
