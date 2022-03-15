package frc.robot.Autonomous;

import frc.robot.Drive.TorDrive;
import frc.robot.Mechanisms.*;
import frc.robot.Mechanisms.MechMaster.autoMech;

public class AutoBall1 {
    public static enum autoRun{
        INIT, Linear1, SHOOT, DONE;
    }

    private autoRun AutoState1 = autoRun.INIT;

    private MechMaster mechMaster;
    private TorDrive torDrive;
    private pivotTrajectory pivot1;
    private linearTrajectory linear1;
    public autoMech AutoMechanics;
    
    public AutoBall1(MechMaster mechMaster, TorDrive torDrive){
        this.torDrive = torDrive;
        this.mechMaster = mechMaster;
        linear1 = new linearTrajectory(torDrive, 3.5, 50);
        //pivot1 = new pivotTrajectory(torDrive, 122.25, 1);

    }

    public void run(){
        switch(AutoState1){
            case INIT:
                linear1.init();
                AutoState1 = autoRun.Linear1;
                break;
            case Linear1:
                linear1.run();
                //mechMaster.autoRun(autoMech.STORE);
                if(linear1.isDone()){
                    //pivot1.init();
                    AutoState1 = autoRun.SHOOT;
                }
                break;
                /*
            case Pivot1:
                pivot1.run();
                mechMaster.autoRun(autoMech.IDLE);
                if (pivot1.isDone()) {
                    AutoState1 = autoRun.SHOOT;
                }
                break;
                */
            case SHOOT:
                mechMaster.autoRun(autoMech.SHOOT);
                break;
            case DONE:
                
                break;
                
        }
    }

}