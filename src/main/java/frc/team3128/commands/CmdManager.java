package frc.team3128.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import common.hardware.input.NAR_XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.team3128.RobotContainer;
import frc.team3128.subsystems.Amper;
import frc.team3128.subsystems.Hopper;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Amper.AmpState;
import frc.team3128.subsystems.Hopper.HopperState;
import frc.team3128.subsystems.Intake.IntakeState;
import frc.team3128.subsystems.Shooter.ShooterState;
import frc.team3128.subsystems.Swerve;
// import frc.team3128.subsystems.Climber;

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

    public static Command ramShoot(boolean once) {
        return sequence(
            amper.setState(AmpState.RETRACTED),
            intake.setState(IntakeState.RETRACTED),
            shooter.setState(ShooterState.SHOOT),
            waitUntil(()-> !shooter.hasObjectPresent()),
            either(
                none(),
                hopper.setState(HopperState.IDLE),
                ()-> once
            )
        );
    }

    // public static Command hopperOuttake() {
    //     return sequence(
    //         intake.setState(IntakeState.RETRACTED),
    //         // TODO: will this slow things down
    //         waitSeconds(0.3),
    //         hopper.runManipulator(HPPR_OUTTAKE_POWER),
    //         waitSeconds(0.35),
    //         hopper.runManipulator(0)
    //     );
    // }
}