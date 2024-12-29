package frc.team3128.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.BooleanSupplier;

import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;

public class SubsystemManager {
    
    public enum RobotState {

        
    }

    private static SubsystemManager instance;

    public static synchronized SubsystemManager getInstance() {
        if (instance == null)
            instance = new SubsystemManager();
        return instance;
    }

    private SubsystemManager() {
        // setName("Robot");
        initShuffleboard();
    }

    // public Command setState(RobotState state, double delay) {
    
        
    //     return sequence(
    //         runOnce(()->Log.info("State", state.toString())),
    //     );
    // }

    // public boolean isState(RobotState state){
    //     return
    // }

    public void initShuffleboard(){
        // for(RobotState state : RobotState.values()){
        //     NAR_Shuffleboard.addData("Robot", state.toString(), ()-> isState(state));
        // }
    }

}
