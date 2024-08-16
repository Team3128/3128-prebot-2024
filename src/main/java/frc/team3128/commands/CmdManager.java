package frc.team3128.commands;

import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.repeatingSequence;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
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
import frc.team3128.subsystems.Climber;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Intake.Setpoint;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Swerve;

public class CmdManager {

    private static Swerve swerve = Swerve.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    private static Climber climber = Climber.getInstance();
    private static Amper amper = Amper.getInstance();
    private static Hopper hopper = Hopper.getInstance();

    private static NAR_XboxController controller = RobotContainer.controller;

    public static Command vibrateController(){
        return new ScheduleCommand(new StartEndCommand(()-> controller.startVibrate(), ()-> controller.stopVibrate()).withTimeout(1));
    }

    /* 
     * dont think it works like this, our robot has only one shooter motor
     * so one shooter motor will ramp up, then the other (WITH NO PID), known as the kicking motor, will push it a little bit to me shot
    */
    public static Command rampUpShoot() {
        return shooter.shoot(SHOOTER_RPM);
    }

    /*
     * will have to change this command to work with new ramp up shoot
     * one possiblity is to keep ramp up shoot in shooter, then have it such that it is split in two. One for ramp up, one for kick, then use it here
     */
    public static Command ramShoot() {
        return sequence(
            rampUpShoot(),
            waitUntil(() -> shooter.atSetpoint()),
            shooter.runKickMotor(KICK_POWER),
            waitSeconds(0.35),
            neutral()
        );
    }

    /*
     * similar logic here
     */
    public static Command autoShoot() {
        return deadline(
            ramShoot(),
            repeatingSequence(
                runOnce(()-> CmdSwerveDrive.setTurnSetpoint(swerve.getTurnAngle(Robot.getAlliance() == Alliance.Red ? focalPointRed : focalPointBlue))),
                waitSeconds(0.1)
            )
        ).andThen(runOnce(() -> CmdSwerveDrive.disableTurn()));
    }

    /*
     * similar logic here
     */
    public static Command rampUpAmp() {
        return sequence(
            shooter.shoot(AMP_RPM),
            amper.extend()
        );
    }

    /*
     * similar logic here
     */
    public static Command amp() {
        return sequence(
            rampUpAmp(),
            waitUntil(() -> shooter.atSetpoint() && amper.atSetpoint()),
            shooter.runKickMotor(KICK_POWER),
            waitSeconds(0.35),
            neutral()
        );
    }

    // TODO: this
    // public static Command autoAmp() {
        
    // }

    public static Command intakeCmd(Setpoint setpoint) {
        return sequence(
            deadline(
                intake.pivotTo(setpoint),
                sequence(
                    intake.runIntakeRollers(),
                    hopper.runManipulator(HOPPER_INTAKE_POWER),
                    waitUntil(()->hopper.hasObjectPresent()),
                    intake.stopRollers(),
                    hopper.runManipulator(0),
                    either(
                        none(),
                        sequence(
                            hopper.runManipulator(HOPPER_INTAKE_POWER),
                            shooter.runKickMotor(HOPPER_INTAKE_POWER),
                            waitUntil(()->shooter.hasObjectPresent()),
                            hopper.runManipulator(0),
                            shooter.runKickMotor(0)
                        ),
                        ()->shooter.hasObjectPresent()
                    )
                )
            ),
            intake.retract()
        );
    }

    public static Command feed(double rpm, double angle) {
        return sequence(
            parallel(
                swerve.turnInPlace(()-> Robot.getAlliance() == Alliance.Blue ? 180-angle : angle).asProxy().withTimeout(1),
                runOnce(()->shooter.startPID(rpm, rpm)),
                waitUntil(()->shooter.atSetpoint())
            ),
            shooter.runKickMotor(KICK_POWER),
            waitSeconds(0.35),
            neutral()
        );
    }

    public static Command neutral() {
        return sequence(
            intake.stopRollers(),
            shooter.setShooter(0),
            shooter.runKickMotor(0),
            amper.retract(),
            hopper.runManipulator(0)
        );
    }
}