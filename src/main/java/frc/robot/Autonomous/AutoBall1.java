package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drive.TorDrive;
import frc.robot.Mechanisms.*;
import frc.robot.Mechanisms.MechMaster.autoMech;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.Timer;

public class AutoBall1 {
    public static enum autoRun{
        INIT, Linear1, PIVOT, SHOOT, DONE;
    }

    private autoRun AutoState1 = autoRun.INIT;

    private MechMaster mechMaster;
    private TorDrive torDrive;
    private pivotTrajectory pivot1;
    private linearTrajectory linear1;
    public autoMech AutoMechanics;
    
    private double PrevTime = Timer.getFPGATimestamp() + 0.2;
    
    public AutoBall1(MechMaster mechMaster, TorDrive torDrive){
        this.torDrive = torDrive;
        this.mechMaster = mechMaster;
        linear1 = new linearTrajectory(torDrive, 3.5f, 5);
        pivot1 = new pivotTrajectory(torDrive, 172.25, 1); //original angle 122.25

    }

    public void run(){
        SmartDashboard.putString("Auto State", AutoState1.toString());
        switch(AutoState1){
            case INIT:
                linear1.init();
                AutoState1 = autoRun.Linear1;
                PrevTime = Timer.getFPGATimestamp();
                
                break;
            case Linear1:
                linear1.run();
                mechMaster.autoRun(autoMech.STORE);
                if(linear1.isDone()){
                    pivot1.init();
                    AutoState1 = autoRun.PIVOT;
                }
                PrevTime = Timer.getFPGATimestamp();
                break;
                
            case PIVOT:
                pivot1.run();
                mechMaster.autoRun(autoMech.IDLE);
                if (pivot1.isDone()) {
                    AutoState1 = autoRun.SHOOT;
                }
                PrevTime = Timer.getFPGATimestamp();
                break;
                
            case SHOOT:
                mechMaster.autoRun(autoMech.SHOOT);
                if (mechMaster.getBallCount() == 0) {
                    AutoState1 = autoRun.DONE;
                }
                PrevTime = Timer.getFPGATimestamp();
                
                break;
                                
            case DONE:
                if (Timer.getFPGATimestamp() < PrevTime + 0.2)
                    mechMaster.autoRun(autoMech.SHOOT);
                else
                    mechMaster.autoRun(autoMech.IDLE);
                break;
                
        }
    }

}