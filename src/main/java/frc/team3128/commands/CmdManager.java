package frc.team3128.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.FocalAimConstants.focalPointBlue;
import static frc.team3128.Constants.FocalAimConstants.focalPointRed;
import static frc.team3128.Constants.HopperConstants.*;
import static frc.team3128.Constants.ShooterConstants.*;

import common.hardware.input.NAR_XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.team3128.Robot;
import frc.team3128.RobotContainer;
import frc.team3128.subsystems.Amper;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Intake.Setpoint;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Swerve;

public class CmdManager {

    private static Swerve swerve = Swerve.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    private static Amper amper = Amper.getInstance();
    private static Hopper hopper = Hopper.getInstance();

    private static NAR_XboxController controller = RobotContainer.controller;

    public static Command vibrateController(){
        return new ScheduleCommand(new StartEndCommand(()-> controller.startVibrate(), ()-> controller.stopVibrate()).withTimeout(1));
    }

    public static Command rampUpShoot() {
        return shooter.shoot(SHOOTER_RPM);
    }

    public static Command kick() {
        return either(
            sequence(
                shooter.runKickMotor(KICK_POWER),
                waitUntil(() -> shooter.noteInRollers()).withTimeout(0.3),
                waitUntil(() -> !shooter.noteInRollers()),
                shooter.runKickMotor(0)
            ),
            none(),
            () -> shooter.noteInKick()
        );
    }

    public static Command kick(boolean once) {
        return sequence(
            kick(),
            either(
                queueNote(),
                sequence(
                    queueNote(),
                    waitUntil(() -> shooter.atSetpoint()),
                    kick()
                ),
                () -> once
            )
        );
    }

    public static Command ramShoot(boolean once) {
        return sequence(
            rampUpShoot(),
            waitUntil(() -> shooter.atSetpoint()),
            kick(once),
            shooter.stopMotors()
        );
    }

    public static Command autoShoot(boolean once) {
        return deadline(
            rampUpShoot(),
            repeatingSequence(
                runOnce(()-> CmdSwerveDrive.setTurnSetpoint(swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue))),
                waitSeconds(0.1)
            )
        ).andThen(ramShoot(once)).andThen(runOnce(() -> CmdSwerveDrive.disableTurn()));
    }

    public static Command rampUpAmp() {
        return sequence(
            shooter.shoot(AMP_RPM),
            amper.extend()
        );
    }

    public static Command amp(boolean once) {
        return sequence(
            rampUpAmp(),
            waitUntil(() -> shooter.atSetpoint() && amper.atSetpoint()),
            kick(once),
            waitSeconds(0.2),
            amper.retract()
        );
    }

    // TODO: this
    // public static Command autoAmp() {
        
    // }

    public static Command intake(Setpoint setpoint) {
        return sequence(
            intake.pivotTo(setpoint),
            intake.runIntakeRollers(),
            hopper.runManipulator(HOPPER_INTAKE_POWER),
            waitUntil(() -> shooter.noteInKick()),
            waitUntil(() -> hopper.noteInFront()),
            stopIntake()
        );
    }

    public static Command stopIntake() {
        return sequence(
            intake.stopRollers(),
            hopper.runManipulator(0),
            intake.retract()
        );
    }

    public static Command feed(double rpm, double angle, boolean once) {
        return sequence(
            parallel(
                swerve.turnInPlace(() -> Robot.getAlliance() == Alliance.Blue ? 180-angle : angle).asProxy().withTimeout(1),
                runOnce(() -> shooter.startPID(rpm)),
                waitUntil(() -> shooter.atSetpoint())
            ),
            kick(once),
            shooter.stopMotors()
        );
    }

    public static Command queueNote() {
        return either(
            none(), 
            sequence(
                hopper.runManipulator(HOPPER_INTAKE_POWER),
                waitUntil(()-> shooter.noteInKick()),
                hopper.runManipulator(0)
            ), 
            () -> (!hopper.hasObjectPresent() || shooter.noteInKick())
        );
    }

    public static Command hopperOuttake() {
        return sequence(
            intake.pivotTo(Setpoint.NEUTRAL),
            // TODO: will this slow things down
            waitUntil(()->intake.atSetpoint()),
            hopper.runManipulator(HOPPER_OUTTAKE_POWER),
            waitSeconds(0.35),
            hopper.runManipulator(0)
        );
    }
}