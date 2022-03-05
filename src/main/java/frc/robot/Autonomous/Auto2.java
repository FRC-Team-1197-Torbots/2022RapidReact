package frc.robot.Autonomous;

import frc.robot.Drive.TorDrive;
import frc.robot.Mechanisms.*;

public class Auto2 {
    public static enum autoRun{
        INIT, Linear1, DONE;
    }

    private autoRun AutoState2 = autoRun.INIT;

    private MechMaster mechMaster;
    private TorDrive torDrive;
    private linearTrajectory linear1;
    
    public Auto2(MechMaster mechMaster, TorDrive torDrive){
        this.torDrive = torDrive;
        this.mechMaster = mechMaster;
        linear1 = new linearTrajectory(torDrive, 5, 3);

    }

    public void run(){
        switch(AutoState2){
            case INIT:
                linear1.init();
                AutoState2 = autoRun.Linear1;
                break;
            case Linear1:
                linear1.run();
                if(linear1.isDone()){
                    AutoState2 = autoRun.DONE;
                }
                break;
            case DONE:
                
                break;
                
        }
    }
}
