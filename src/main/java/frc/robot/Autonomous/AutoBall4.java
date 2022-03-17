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
        INIT, LINEAR1, PIVOT1, SHOOT1, LINEAR2, SHOOT2,
        PIVOT2, LINEAR3, PIVOT3, LINEAR4, SHOOT3, DONE;
    }


private autoRun AutoState1 = autoRun.INIT;

    private MechMaster mechMaster;
    private TorDrive torDrive;
    private linearTrajectory2 linear1;
    private pivotTrajectory pivot1;
    private linearTrajectory2 linear2;
    private pivotTrajectory pivot2;
    private linearTrajectory2 linear3;
    private pivotTrajectory pivot3;
    private linearTrajectory2 linear4;
    public autoMech AutoMechanics;
    
    private double PrevTime = Timer.getFPGATimestamp() + 0.2;
    
    public AutoBall4(MechMaster mechMaster, TorDrive torDrive){
        this.torDrive = torDrive;
        this.mechMaster = mechMaster;
        linear1 = new linearTrajectory2(torDrive, 3.5f, 1.5);
        pivot1 = new pivotTrajectory(torDrive, 110, 1.5);
        linear2 = new linearTrajectory2(torDrive, 10.0f, 2.5);
        pivot2 = new pivotTrajectory(torDrive, -37, 1.5);
        linear3 = new linearTrajectory2(torDrive, 9.8, 3.5);
        pivot3 = new pivotTrajectory(torDrive, 180, 1.5);
        linear4 = new linearTrajectory2(torDrive, 20, 1.0);
    }

    public void run(){
        SmartDashboard.putString("Auto State", AutoState1.toString());
        switch(AutoState1){
            case INIT:
                linear1.init();
                PrevTime = Timer.getFPGATimestamp();
                AutoState1 = autoRun.LINEAR1;
                
                break;
            case LINEAR1:
                linear1.run();
                mechMaster.autoRun(autoMech.STORE, turretMech.SET, 45);
                if((linear1.isDone()) 
                || (Timer.getFPGATimestamp() - PrevTime > 1.5)){
                    pivot1.init();
                    torDrive.setMotorSpeeds(0, 0);
                    AutoState1 = autoRun.PIVOT1;
                }
                break;
                
            case PIVOT1:
                pivot1.run();
                mechMaster.autoRun(autoMech.IDLE, turretMech.SET, 45);
                if (pivot1.isDone()) {
                    PrevTime = Timer.getFPGATimestamp();
                    torDrive.setMotorSpeeds(0, 0);
                    AutoState1 = autoRun.SHOOT1;
                }
                break;
                
            case SHOOT1:
                mechMaster.autoRun(autoMech.SHOOT, turretMech.AUTOAIM, 0);
                if ((Timer.getFPGATimestamp() - PrevTime > 2)) {
                    linear2.init();
                    torDrive.setMotorSpeeds(0, 0);
                    AutoState1 = autoRun.LINEAR2;
                    PrevTime = Timer.getFPGATimestamp();
                }
                
                break;
                
            case LINEAR2:
                if (Timer.getFPGATimestamp() < PrevTime + 0.4) {
                    mechMaster.autoRun(autoMech.SHOOT, turretMech.AUTOAIM, 0);
                } else {
                    linear2.run();
                    mechMaster.autoRun(autoMech.STORE, turretMech.SET, 120f);

                    if(linear2.isDone()){
                        PrevTime = Timer.getFPGATimestamp();
                        torDrive.setMotorSpeeds(0, 0);
                        AutoState1 = autoRun.SHOOT2;
                    }
                }

                break;
                
            case SHOOT2:
                mechMaster.autoRun(autoMech.SHOOT, turretMech.AUTOAIM, 0);
                // ((Timer.getFPGATimestamp() - PrevTime) > 2)) {
                if((Timer.getFPGATimestamp() - PrevTime) > 2) {
                    PrevTime = Timer.getFPGATimestamp();
                    pivot2.init();
                    torDrive.setMotorSpeeds(0, 0);
                    AutoState1 = autoRun.PIVOT2;
                }
                
                break;
            
            case PIVOT2:
                if (Timer.getFPGATimestamp() < PrevTime + 0.5) {
                    mechMaster.autoRun(autoMech.SHOOT, turretMech.AUTOAIM, 0);
                } else {
                    pivot2.run();
                    mechMaster.autoRun(autoMech.STORE, turretMech.SET, 0);

                    if (pivot2.isDone()) {
                        linear3.init();
                        torDrive.setMotorSpeeds(0, 0);
                        AutoState1 = autoRun.LINEAR3;
                    }
                }
                    
                
                break;

            case LINEAR3:
                linear3.run();
                mechMaster.autoRun(autoMech.STORE, turretMech.SET, 0);
                if(linear3.isDone()){
                    pivot3.init();
                    torDrive.setMotorSpeeds(0, 0);
                    AutoState1 = autoRun.PIVOT3;
                }
                
                break;

            case PIVOT3:
                pivot3.run();
                mechMaster.autoRun(autoMech.STORE, turretMech.SET, 0);
                if (pivot3.isDone()) {
                    torDrive.setMotorSpeeds(0, 0);
                    linear4.init();
                    PrevTime = Timer.getFPGATimestamp();
                    AutoState1 = autoRun.LINEAR4;
                }
                break;
            case LINEAR4:
                linear4.run();
                mechMaster.autoRun(autoMech.STORE, turretMech.AUTOAIM, 0);
                if(linear4.isDone()) {
                    PrevTime = Timer.getFPGATimestamp();
                    AutoState1 = autoRun.SHOOT3;
                }
                
                break;

            case SHOOT3:
                mechMaster.autoRun(autoMech.SHOOT, turretMech.AUTOAIM, 0);
                if (Timer.getFPGATimestamp() - PrevTime > 4
                && (Timer.getFPGATimestamp() - PrevTime > 0.5)) {
                    PrevTime = Timer.getFPGATimestamp();
                    torDrive.setMotorSpeeds(0, 0);
                    AutoState1 = autoRun.DONE;
                }
                
                break;
                                
            case DONE:
                if (Timer.getFPGATimestamp() < PrevTime + 0.2)
                    mechMaster.autoRun(autoMech.SHOOT, turretMech.AUTOAIM, 0);
                else
                    mechMaster.autoRun(autoMech.IDLE, turretMech.AUTOAIM, 0);
                break;
                
        }
    }
}