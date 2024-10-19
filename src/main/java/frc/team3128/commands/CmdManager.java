package frc.team3128.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.FocalAimConstants.focalPointBlue;
import static frc.team3128.Constants.FocalAimConstants.focalPointRed;
import static frc.team3128.Constants.FieldConstants.*;
import static frc.team3128.Constants.HopperConstants.*;
import static frc.team3128.Constants.ShooterConstants.*;

import common.hardware.input.NAR_XboxController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.team3128.Robot;
import frc.team3128.RobotContainer;
import frc.team3128.Constants.ShooterConstants;
import frc.team3128.subsystems.Amper;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Amper.AmpState;
// import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Intake.IntakeState;

public class CmdManager {

    private static Swerve swerve = Swerve.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    private static Amper amper = Amper.getInstance();
    private static Hopper hopper = Hopper.getInstance();
    //private static Climber climber = Climber.getInstance();

    private static NAR_XboxController controller = RobotContainer.controller;

    public static Command vibrateController(){
        return new ScheduleCommand(new StartEndCommand(()-> controller.startVibrate(), ()-> controller.stopVibrate()).withTimeout(1));
    }

    public static Command kick() {
        return either(
            sequence(
                shooter.runKickMotor(KICK_POWER),
                waitUntil(() -> !shooter.noteInRollers()).withTimeout(0.3),
                shooter.runKickMotor(0)
            ),
            none(),
            () -> shooter.noteInRollers()
        );
    }

    public static Command kick(boolean once) {
        return sequence(
            kick(),
            either(
                // queueNote(),
                none(),
                sequence(
                    // queueNote(),
                    waitSeconds(0.3),
                    kick()
                ),
                () -> once
            )
        );
    }

    public static Command ramShootTriggerless(){
        return sequence(
            shooter.runShooter(0.8),
            waitSeconds(0.5),
            shooter.runKickMotor(KICK_POWER),
            waitSeconds(1),
            shooter.stopMotors(),
            runOnce(()->shooter.disable())
        );
    }


    public static Command ramShoot(boolean once) {
        return sequence(
            ramShootNoStop(once),
            shooter.stopMotors()
        );
    }

    public static Command ramShoot() {
        return sequence(
            shooter.runKickMotor(.5)
        );
    }

    public static Command ramShootNoStop(boolean once) {
        return sequence(
            shooter.rampUpShooter(),
            waitUntil(()->shooter.atSetpoint()),
            kick(once)
        );
    }

    public static Command autoShoot() {
        return sequence(
            shooter.rampUpShooter(),
            shooter.setShooting(true));
    }

    // public static Command autoShoot(boolean once) {
    //     return deadline(
    //         rampUpShoot(),
    //         repeatingSequence(
    //             runOnce(()-> CmdSwerveDrive.setTurnSetpoint(swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue))),
    //             waitSeconds(0.1)
    //         )
    //     ).andThen(ramShoot(once)).andThen(runOnce(() -> CmdSwerveDrive.disableTurn()));
    // }

    public static Command intake() {
        return intake.setState(IntakeState.GROUND);
        // return sequence(
        //     intake.runIntakeRollers(),
        //     intake.pivotTo(setpoint)
        // );
    }

    public static Command retractIntake() {
        return intake.setState(IntakeState.NEUTRAL);
        // return sequence(
        //     intake.stopRollers(),
        //     // hopper.runManipulator(0),
        //     intake.retract()
        // );
    }

    // public static Command feed(double rpm, double angle, boolean once) {
    //     return sequence(
    //         parallel(
    //             swerve.turnInPlace(() -> Robot.getAlliance() == Alliance.Blue ? 180-angle : angle).asProxy().withTimeout(1),
    //             runOnce(() -> shooter.startPID(rpm)),
    //             waitUntil(() -> shooter.atSetpoint())
    //         ),
    //         kick(once),
    //         shooter.stopMotors()
    //     );
    // }

    public static Command outtake(){
        return sequence(
            shooter.runKickMotor(-1),
            hopper.runManipulator(-1),
            intake.setState(IntakeState.OUTTAKE)
        );
    }

    public static Command stop(){
        return sequence(
            shooter.runKickMotor(0),
            hopper.runManipulator(0),
            retractIntake()
        );
    }

    public static Command hopperOuttake() {
        return sequence(
            retractIntake(),
            // TODO: will this slow things down
            waitSeconds(0.3),
            hopper.runManipulator(HOPPER_OUTTAKE_POWER),
            waitSeconds(0.35),
            hopper.runManipulator(0)
        );
    }

    public static Command ampFinAndDown(){
        return sequence(
            amper.setState(AmpState.EXTENDED),
            waitUntil(() -> amper.atSetpoint()),
            waitSeconds(0.25),
            shooter.setShooting(true)
            // waitUntil(() -> !shooter.getShooting())
            // waitSeconds(1),
            // amper.runRollers(0),
            // amper.moveElevator(0)
        );
    }

    public static Command ejectNote(){
        CommandScheduler.getInstance().cancel(CommandScheduler.getInstance().requiring(hopper));
        CommandScheduler.getInstance().cancel(CommandScheduler.getInstance().requiring(intake));
        return sequence(
            retractIntake(),
            waitSeconds(0.3),
            hopper.runManipulator(HOPPER_OUTTAKE_POWER),
            waitSeconds(0.2),
            hopper.runManipulator(0)
        );
    }

    // public static Command feed(double rpm, double angle){
    //     return sequence(
    //         shooter.runShooter(rpm),
    //         swerve.turnInPlace(()-> angle),
    //         shooter.runKickMotor(KICK_POWER),
    //         hopper.intake(),
    //         waitSeconds(1),
    //         shooter.stopMotors(),
    //         hopper.runManipulator(0)
    //     );
    // }

    //better feed logic UNTESTED
    public static Command feed(double shooterPower, double angle){
        return sequence(
            parallel(
                swerve.turnInPlace(()-> allianceFlip(Rotation2d.fromDegrees(angle)).getDegrees()),
                shooter.runShooter(shooterPower)
            ),
            shooter.runKickMotor(KICK_POWER),
            waitSeconds(0.25),
            hopper.runManipulator(HOPPER_INTAKE_POWER),
            waitUntil(()-> !shooter.noteInRollers() || !hopper.hasObjectPresent()),
            waitSeconds(0.5),
            shooter.stopMotors(),
            hopper.runManipulator(0)
        );
    }
}