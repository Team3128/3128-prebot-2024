// package frc.team3128.subsystems;

// import common.core.controllers.TrapController;
// import common.core.subsystems.ElevatorTemplate;
// import common.hardware.motorcontroller.NAR_CANSpark;
// import common.hardware.motorcontroller.NAR_CANSpark.SparkMaxConfig;
// import common.hardware.motorcontroller.NAR_Motor.Neutral;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj.PWM;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import static edu.wpi.first.wpilibj2.command.Commands.*;

// import static frc.team3128.Constants.ClimberConstants.*;

// public class Climber extends SubsystemBase {
//     public class ClimberElevator extends ElevatorTemplate {
//         private ClimberElevator() {
//             super(new TrapController(PIDConstants, TRAP_CONSTRAINTS), climberMotor);

//             // TODO: figure out kG function
//             // setkG_Function(());
//             setTolerance(POSITION_TOLERANCE);
//             setConstraints(MIN_SETPOINT, MAX_SETPOINT);
//             initShuffleboard();
//         }
//         @Override
//         protected void configMotors() {
//             // TODO: figure unit conversion out
//             climberMotor.setUnitConversionFactor(UNIT_CONV_FACTOR);
//             climberMotor.setCurrentLimit(CURRENT_LIMIT);

//             //TODO: set back to BRAKE ONCE PID DONE
//             climberMotor.setNeutralMode(Neutral.COAST);

//             climberMotor.setStatusFrames(SparkMaxConfig.POSITION);
//         }
//         @Override
//         public boolean atSetpoint(){
//             return Math.abs(climberMotor.getPosition() - getSetpoint()) < POSITION_TOLERANCE;
//         }
//     }

//     public enum Setpoint {
//         EXTENDED(70),
//         RETRACTED(0);

//         private double setpoint;
//         private Setpoint(double setpoint) {
//             this.setpoint = setpoint;
//         }
//     }

//     private final NAR_CANSpark climberMotor = new NAR_CANSpark(CLIMB_MOTOR_ID);
//     private final PWM SERVO = new PWM(PWM_SERVO_ID);
//     private static Climber instance;
//     private double currentSetpoint = Setpoint.RETRACTED.setpoint;
//     private ClimberElevator climberElevator;

//     public Climber() {
//         climberElevator = new ClimberElevator();
//     }

//     public static synchronized Climber getInstance() {
//         if (instance == null)
//             instance = new Climber();
//         return instance;
//     }

//     public void setSetpoint(Setpoint setpoint) {
//         currentSetpoint = setpoint.setpoint;
//     }

//     public double getSetpoint() {
//         return currentSetpoint;
//     }

//     public Command extendElevator() {
//         return sequence(
//             runOnce(()-> SERVO.setPosition(1)),
//             runOnce(()-> setSetpoint(Setpoint.EXTENDED)),
//             waitSeconds(0.2),
//             runOnce(()->climberMotor.set(0.7)),
//             waitUntil(()-> climberElevator.atSetpoint()),
//             runOnce(()->climberMotor.set(0))
//         );
//     }

//     public Command retractElevator() {
//         return sequence(
//             runOnce(()-> SERVO.setPosition(0)),
//             runOnce(()-> setSetpoint(Setpoint.RETRACTED)),
//             waitSeconds(0.2),
//             runOnce(()->climberMotor.set(-0.7)),
//             waitUntil(()-> climberElevator.atSetpoint()),
//             runOnce(()->climberMotor.set(0))
//         );
//     }
// }
