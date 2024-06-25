package frc.team3128.subsystems;

import common.core.subsystems.ManipulatorTemplate;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.team3128.Constants.HopperConstants.*;

public class Hopper extends ManipulatorTemplate {

    private static Hopper instance;
    private DigitalInput limitSwitch;

    private Hopper() {
        super(STALL_CURRENT, INTAKE_POWER, OUTTAKE_POWER, STALL_POWER, 0.3, HOPPER_MOTOR);
        limitSwitch = new DigitalInput(1);
        initShuffleboard();
    }

    public static synchronized Hopper getInstance() {
        if (instance == null) 
            instance = new Hopper();
        return instance;
    }

    @Override
    protected void configMotors() {
        HOPPER_MOTOR.setInverted(false);
        HOPPER_MOTOR.enableVoltageCompensation(9);
        HOPPER_MOTOR.setNeutralMode(Neutral.BRAKE);
        HOPPER_MOTOR.setCurrentLimit(CURRENT_LIMIT);
        HOPPER_MOTOR.setStatusFrames(SparkMaxConfig.VELOCITY);
    }

    @Override
    public boolean hasObjectPresent() {
        return !limitSwitch.get();
    }
}
