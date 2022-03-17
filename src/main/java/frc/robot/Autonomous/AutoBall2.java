

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Autonomous.curveTrajectory.Direction;
import frc.robot.Drive.TorDrive;
import frc.robot.Mechanisms.*;
import frc.robot.Mechanisms.MechMaster.autoMech;

public class AutoBall2 {
    public static enum autoRun{
        INIT, Linear1, Linear2, Linear3, Curve1, DONE;
    }

    private autoRun AutoState2 = autoRun.INIT;

    private MechMaster mechMaster;
    private TorDrive torDrive;
    private linearTrajectory2 linear1;
    private linearTrajectory2 linear2;
    private linearTrajectory2 linear3;
    private curveTrajectory curve1;
    //private pivotTrajetory pivot1;
    private double startTime;
    private double currentTime;

    private double currentPosition;
    private double lastPosition;

    private final double curve_outer_distance = (2f * Math.PI * 37.375) / 36f;

    public AutoBall2(MechMaster mechMaster, TorDrive torDrive){
        this.torDrive = torDrive;
        this.mechMaster = mechMaster;
        linear1 = new linearTrajectory2(torDrive, 10, 10);
        linear2 = new linearTrajectory2(torDrive, 0, 10);
        //linear3 = new linearTrajectory(torDrive, 0.5, 10);
        //curve1 = new curveTrajectory(torDrive, curve_outer_distance, 5, Direction.RIGHT);

    }

    public void run(){
        currentTime = Timer.getFPGATimestamp();
        switch(AutoState2){
            case INIT:
                linear1.init();
                //mechMaster.autoRun(autoMech.IDLE);
                AutoState2 = autoRun.Linear1;
                break;
            case Linear1:
                linear1.run();
                //mechMaster.autoRun(autoMech.IDLE);
                if(linear1.isDone()){
                    linear2.init();
                    AutoState2 = autoRun.Linear2;
                }
                break;
            
            
            case Linear2:
                linear2.run();
                // mechMaster.autoRun(autoMech.IDLE);
                if(linear2.isDone()){
                    //linear3.init();
                    AutoState2 = autoRun.DONE;
                }
                break;
                /*
            case Linear3:
                linear3.run();
                //mechMaster.autoRun(autoMech.IDLE);
                if(linear3.isDone()){
                    AutoState2 = autoRun.DONE;
                }
                break;
                */

            case DONE:
                break;
                
        }
    }
}
