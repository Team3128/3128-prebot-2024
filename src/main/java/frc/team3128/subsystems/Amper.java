package frc.team3128.subsystems;

import static frc.team3128.Constants.AmperConstants.*;

import common.core.controllers.TrapController;
import common.core.subsystems.ElevatorTemplate;
import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
import common.hardware.motorcontroller.NAR_Motor.Control;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

public class Amper extends ElevatorTemplate {
    
    public enum Setpoint {
        //TODO: correct EXTEND setpoints
        FULLEXTEND(21.25),
        PARTEXTEND(21.25*0.8),
        RETRACTED(0);

        private double setpoint;
        private Setpoint(double setpoint) {
            this.setpoint = setpoint;
        }
    }

    private static Amper instance;

    public static synchronized Amper getInstance() {
        if (instance == null)
            instance = new Amper();
        return instance;
    }

    private Amper() {
        super(new TrapController(PIDConstants, TRAP_CONSTRAINTS), ELEV_MOTOR);

        //TODO: remove once done testing
        // this.setSafetyThresh(100);
        
        // setkG_Function(() ->  getMeasurement()*Math.sin(AMPER_ANGLE));

        setTolerance(POSITION_TOLERANCE);
        setConstraints(MIN_SETPOINT, MAX_SETPOINT);
        // initShuffleboard();
    }

    @Override
    protected void configMotors() {
        ELEV_MOTOR.setUnitConversionFactor(UNIT_CONV_FACTOR);
        ELEV_MOTOR.setCurrentLimit(CURRENT_LIMIT);
        ELEV_MOTOR.setInverted(true);

        ELEV_MOTOR.setNeutralMode(Neutral.COAST);
        ROLLER_MOTOR.setNeutralMode(Neutral.COAST);

        ELEV_MOTOR.setStatusFrames(SparkMaxConfig.POSITION);
        ROLLER_MOTOR.setDefaultStatusFrames();
    }
    
    public void setVoltage(double volts) {
        ELEV_MOTOR.set(0, Control.Position);
        ELEV_MOTOR.setVolts(volts);
    }

    public double getVelocity() {
        return ELEV_MOTOR.getVelocity();
    }
    
    public double getPosition() {
        return ELEV_MOTOR.getPosition();
    }

    @Override
    public void initShuffleboard(){
        super.initShuffleboard();

        NAR_Shuffleboard.addData(getName(), "Measurement2", ()-> getPosition(), 5, 1);
        NAR_Shuffleboard.addData(getName(), "Current", ()-> ELEV_MOTOR.getStallCurrent(), 5, 2);
        NAR_Shuffleboard.addData(getName(), "Voltage", ()-> ELEV_MOTOR.getMotor().getBusVoltage(), 5, 3);
        NAR_Shuffleboard.addData(getName(), "Velocity", ()-> getVelocity(), 1,0);
    }

    public Command runRollers() {
        return runRollers(ROLLER_POWER);
    }

    public Command stopRollers(){
        return runRollers(0);
    }

    public Command runRollers(double power) {
        return runOnce(() -> ROLLER_MOTOR.set(power));
    }  

    public Command partExtend() {
        return moveElevator(Setpoint.PARTEXTEND.setpoint);
    }

    public Command fullExtend(){
        return moveElevator(Setpoint.FULLEXTEND.setpoint);
    }

    public Command retract() {
        return moveElevator(Setpoint.RETRACTED.setpoint);
    }

}