package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drive.TorDrive;
import frc.robot.Mechanisms.*;
import frc.robot.Mechanisms.MechMaster.autoMech;
import frc.robot.Mechanisms.MechMaster.turretMech;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.Timer;

public class AutoBall4 {
    public enum autoRun {
        INIT, LINEAR1, LINEUP1, SHOOT1, PIVOT1, LINEAR2, LINEAR3, PAUSE, SHOOT2, DONE;
    }


private autoRun AutoState1 = autoRun.INIT;

    private MechMaster mechMaster;
    private TorDrive torDrive;
    private linearTrajectory2 linear1;
    private pivotTrajectory pivot1;
    private linearTrajectory2 linear2;
    private pivotTrajectory pivot2;
    private linearTrajectoryBack linear3;
    private pivotTrajectory pivot3;
    public autoMech AutoMechanics;
    
    private double PrevTime = Timer.getFPGATimestamp() + 0.2;
    
    public AutoBall4(MechMaster mechMaster, TorDrive torDrive){
        this.torDrive = torDrive;
        this.mechMaster = mechMaster;
        linear1 = new linearTrajectory2(torDrive, 4, 1); //4.0 2.5
        pivot1 = new pivotTrajectory(torDrive, 13, 0.75); //10, 0.75
        linear2 = new linearTrajectory2(torDrive, 13.5, 3.0); //13.67, 3.0 //12.67 4.0 //4.0 //3.5

        linear3 = new linearTrajectoryBack(torDrive, -5, 1.5); //-6 1.5 //-4, 1.0 //1.75 original

    }

    public void run(){
        //System.out.println(AutoState1.toString());

        //SmartDashboard.putString("Auto State", AutoState1.toString());
        //SmartDashboard.putNumber("Gyro value: ", torDrive.getHeading());
        //SmartDashboard.putNumber("Position", torDrive.getPosition());
        switch(AutoState1){
            case INIT:
                linear1.init();
                PrevTime = Timer.getFPGATimestamp();
                AutoState1 = autoRun.LINEAR1;
                
                break;
            case LINEAR1:
                linear1.run();
                mechMaster.autoRun(autoMech.STORE, turretMech.SET, 170);                
                if((linear1.isDone())){
                    PrevTime = Timer.getFPGATimestamp();
                    torDrive.setMotorSpeeds(0, 0);
                    AutoState1 = autoRun.LINEUP1;
                }
                break;
            
            case LINEUP1:
                if (Timer.getFPGATimestamp() - PrevTime > 0.5)
                    AutoState1 = autoRun.SHOOT1;
                else 
                    mechMaster.autoRun(autoMech.STORE, turretMech.AUTOAIM, 0);                    
                break;
                
            case SHOOT1:
                // if((Timer.getFPGATimestamp() - PrevTime < 1)) {
                //     mechMaster.autoRun(autoMech.IDLE, turretMech.SET, 170);
                // } else {
                mechMaster.autoRun(autoMech.SHOOT, turretMech.AUTOAIM, 0);
                // }
                if ((Timer.getFPGATimestamp() - PrevTime > 3)) {
                    pivot1.init();
                    torDrive.setMotorSpeeds(0, 0);
                    AutoState1 = autoRun.PIVOT1;
                }
                
                break;

            case PIVOT1:
                pivot1.run();
                mechMaster.autoRun(autoMech.IDLE, turretMech.AUTOAIM, 170);
                if (pivot1.isDone()) {
                    linear2.init();
                    torDrive.setMotorSpeeds(0, 0);
                    AutoState1 = autoRun.LINEAR2;
                }
                break;
            case LINEAR2:
                linear2.run();
                mechMaster.autoRun(autoMech.STORE, turretMech.SET, 170);

                if(linear2.isDone()){
                    linear3.init();
                    torDrive.setMotorSpeeds(0, 0);
                    AutoState1 = autoRun.LINEAR3;
                }
                break;

            case LINEAR3:
                linear3.run();
                mechMaster.autoRun(autoMech.REVUP, turretMech.AUTOAIM, 170);
                if(linear3.isDone()){
                    PrevTime = Timer.getFPGATimestamp();
                    torDrive.setMotorSpeeds(0, 0);
                    AutoState1 = autoRun.PAUSE;
                }
                
                break;
            
            case PAUSE:
                mechMaster.autoRun(autoMech.REVUP, turretMech.AUTOAIM, 170);
                if (Timer.getFPGATimestamp() - PrevTime > 1)
                    AutoState1 = autoRun.SHOOT2;
                break;

            case SHOOT2:
                mechMaster.autoRun(autoMech.SCUFF_SHOOT, turretMech.AUTOAIM, 0);
                
                if (Timer.getFPGATimestamp() - PrevTime > 3.5) {
                    PrevTime = Timer.getFPGATimestamp();
                    torDrive.setMotorSpeeds(0, 0);
                    AutoState1 = autoRun.DONE;
                }                              
                break;
            

            case DONE:
                if (Timer.getFPGATimestamp() < PrevTime + 0.2)
                    mechMaster.autoRun(autoMech.SHOOT, turretMech.AUTOAIM, 0);
                else
                    mechMaster.autoRun(autoMech.IDLE, turretMech.SET, 0);
                break;
                
        }
    }

}