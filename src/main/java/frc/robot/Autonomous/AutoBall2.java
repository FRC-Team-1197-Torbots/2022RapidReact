

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Autonomous.curveTrajectory.Direction;
import frc.robot.Drive.TorDrive;
import frc.robot.Mechanisms.*;
import frc.robot.Mechanisms.MechMaster.autoMech;

public class AutoBall2 {
    public static enum autoRun{
        INIT, Linear1, Linear2, Curve1, DONE;
    }

    private autoRun AutoState2 = autoRun.INIT;

    private MechMaster mechMaster;
    private TorDrive torDrive;
    private linearTrajectory linear1;
    private linearTrajectory linear2;
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
        linear1 = new linearTrajectory(torDrive, 3.5, 3);
        linear2 = new linearTrajectory(torDrive, 2.78613941667, 3);
        curve1 = new curveTrajectory(torDrive, curve_outer_distance, 5, Direction.RIGHT);

    }

    public void run(){
        currentTime = Timer.getFPGATimestamp();
        switch(AutoState2){
            case INIT:
                linear1.init();
                mechMaster.autoRun(autoMech.IDLE);
                AutoState2 = autoRun.Linear1;
                break;
            case Linear1:
                linear1.run();
                mechMaster.autoRun(autoMech.STORE);
                if(linear1.isDone()){
                    curve1.init();
                    AutoState2 = autoRun.Curve1;
                }
                break;
            case Curve1:
                curve1.run();
                mechMaster.autoRun(autoMech.SHOOT);
                if(curve1.isDone()){
                    linear2.init();
                    AutoState2 = autoRun.Linear2;
                }
                break;
            case Linear2:
                
        }
    }
}
