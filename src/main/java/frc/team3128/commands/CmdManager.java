package frc.team3128.commands;

import common.hardware.input.NAR_XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.team3128.RobotContainer;
import frc.team3128.subsystems.Swerve;
// import frc.team3128.subsystems.Climber;

public class CmdManager {

    private static Swerve swerve = Swerve.getInstance();

    //private static Climber climber = Climber.getInstance();

    private static NAR_XboxController controller = RobotContainer.controller;

    public static Command vibrateController(){
        return new ScheduleCommand(new StartEndCommand(()-> controller.startVibrate(), ()-> controller.stopVibrate()).withTimeout(1));
    }
}