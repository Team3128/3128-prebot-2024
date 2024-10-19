// package frc.team3128.subsystems;

// import common.core.controllers.TrapController;
// import common.core.subsystems.ElevatorTemplate;
// import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
// import common.hardware.motorcontroller.NAR_Motor.Neutral;
// import edu.wpi.first.wpilibj2.command.Command;

// import common.core.controllers.Controller;
// import common.core.controllers.TrapController;
// import common.core.subsystems.ManipulatorTemplate;
// import common.core.subsystems.PivotTemplate;
// import common.core.subsystems.ShooterTemplate;
// import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
// import common.hardware.motorcontroller.NAR_Motor.Neutral;
// import common.utility.shuffleboard.NAR_Shuffleboard;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import static edu.wpi.first.wpilibj2.command.Commands.*;

// import static frc.team3128.Constants.ClimberConstants.*;

// public class Climber extends ElevatorTemplate {

//     public enum Setpoint {
//         EXTENDED(70),
//         RETRACTED(0);

//         private double setpoint;
//         private Setpoint(double setpoint) {
//             this.setpoint = setpoint;
//         }
//     }

//     private static Climber instance;
//     private double currentSetpoint = Setpoint.RETRACTED.setpoint;

//     public static synchronized Climber getInstance() {
//         if (instance == null)
//             instance = new Climber();
//         return instance;
//     }

//     private Climber() {
//         super(new TrapController(PIDConstants, TRAP_CONSTRAINTS), CLIMB_MOTOR);

//         // TODO: figure out kG function
//         // setkG_Function(());
//         setTolerance(POSITION_TOLERANCE);
//         setConstraints(MIN_SETPOINT, MAX_SETPOINT);
//         initShuffleboard();
//     }

//     public void setSetpoint(Setpoint setpoint) {
//         currentSetpoint = setpoint.setpoint;
//     }

//     public double getSetpoint() {
//         return currentSetpoint;
//     }

//     @Override
//     protected void configMotors() {
//         // TODO: figure unit conversion out
//         CLIMB_MOTOR.setUnitConversionFactor(UNIT_CONV_FACTOR);
//         CLIMB_MOTOR.setCurrentLimit(CURRENT_LIMIT);

//         //TODO: set back to BRAKE ONCE PID DONE
//         CLIMB_MOTOR.setNeutralMode(Neutral.COAST);

//         CLIMB_MOTOR.setStatusFrames(SparkMaxConfig.POSITION);
//     }

//     @Override
//     public boolean atSetpoint(){
//         return Math.abs(CLIMB_MOTOR.getPosition() - getSetpoint()) < POSITION_TOLERANCE;
//     }

//     public Command extendElevator() {
//         return sequence(
//             runOnce(()-> SERVO.setPosition(1)),
//             runOnce(()-> setSetpoint(Setpoint.EXTENDED)),
//             waitSeconds(0.2),
//             runOnce(()->CLIMB_MOTOR.set(0.7)),
//             waitUntil(()-> atSetpoint()),
//             runOnce(()->CLIMB_MOTOR.set(0))
//         );
//     }

//     public Command retractElevator() {
//         return sequence(
//             runOnce(()-> SERVO.setPosition(0)),
//             runOnce(()-> setSetpoint(Setpoint.RETRACTED)),
//             waitSeconds(0.2),
//             runOnce(()->CLIMB_MOTOR.set(-0.7)),
//             waitUntil(()-> atSetpoint()),
//             runOnce(()->CLIMB_MOTOR.set(0))
            
//         );
//     }

// }
