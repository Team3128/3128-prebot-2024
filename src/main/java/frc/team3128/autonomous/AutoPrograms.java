package frc.team3128.autonomous;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;

import common.utility.shuffleboard.NAR_Shuffleboard;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Mason Lam
 */

public class AutoPrograms {
    
    private HashMap<String, Command> autoMap = new HashMap<String, Command>();
    private final SendableChooser<Command> autoChooser;

    public AutoPrograms() {
        Trajectories.initTrajectories();
        initAutoSelector();
        autoChooser = new SendableChooser<Command>();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void initAutoSelector() {
        

        final String[] autoStrings = new String[] {
        };
        
        // NarwhalDashboard.getInstance().addAutos(autoStrings);
        for (final String auto : autoStrings) {
            if (auto.equals("default")) continue;
            autoMap.put(auto, Trajectories.getPathPlannerAuto(auto));
        }
    }

    public Command getAutonomousCommand() {
        // String selectedAutoName = NarwhalDashboard.getInstance().getSelectedAuto();
        String selectedAutoName = "";
        
        if (selectedAutoName.equals("")) {
            return autoChooser.getSelected();
        }
        else if (selectedAutoName.equals("default")) {
            return defaultAuto();
        }

        return autoMap.get(selectedAutoName).beforeStarting(Trajectories.resetAuto());
    }

    private Command defaultAuto(){
        return none();
    }
}