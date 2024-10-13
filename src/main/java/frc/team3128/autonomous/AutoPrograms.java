package frc.team3128.autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.Robot;
import frc.team3128.subsystems.Shooter;
import frc.team3128.subsystems.Swerve;
import common.utility.narwhaldashboard.NarwhalDashboard;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.Constants.ShooterConstants.KICK_POWER;
import static frc.team3128.Constants.ShooterConstants.SHOOTER_RPM;
import static frc.team3128.commands.CmdManager.ramShootTriggerless;

import java.util.HashMap;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Mason Lam
 */

public class AutoPrograms {

    private HashMap<String, Command> autoMap = new HashMap<String, Command>();

    public AutoPrograms() {

        Trajectories.initTrajectories();
        initAutoSelector();
    }

    private void initAutoSelector() {
        final String[] autoStrings = new String[] {
            "middle_FU",
            "top-3Rush"
        };
        
        NarwhalDashboard.getInstance().addAutos(autoStrings);
        for (final String auto : autoStrings) {
            if (auto.equals("default")) continue;
            autoMap.put(auto, Trajectories.getPathPlannerAuto(auto));
        }
    }

    public Command getAutonomousCommand() {
        // String selectedAutoName = NarwhalDashboard.getInstance().getSelectedAuto();
        String hardcode = "default";
        //"" = no auto
        //"default" just shoots
        //"middle_FU"
        //"top-3Rush"
        
        Command autoCommand;
        if (hardcode.equals("")) {
            return none();
        }
        else if (hardcode.equals("default")) {
            return defaultAuto();
        }
        autoCommand = autoMap.get(hardcode);

        return autoCommand.beforeStarting(Trajectories.resetAuto());
    }

    private Command defaultAuto(){
        return ramShootTriggerless();
    }
}