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
        INIT, LINEAR1, PIVOT1, SHOOT1, PIVOT2, LINEAR2, PIVOT3,
        LINEAR3, SHOOT2, DONE;
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
    public autoMech AutoMechanics;
    
    private double PrevTime = Timer.getFPGATimestamp() + 0.2;
    
    public AutoBall4(MechMaster mechMaster, TorDrive torDrive){
        this.torDrive = torDrive;
        this.mechMaster = mechMaster;
        linear1 = new linearTrajectory2(torDrive, 4.0, 2);
        pivot1 = new pivotTrajectory(torDrive, 50, 1.3);
        pivot2 = new pivotTrajectory(torDrive, -40, 1.00); //-41 then -35 then -40
        linear2 = new linearTrajectory2(torDrive, 12.67, 3.5); //12.0
        
        pivot3 = new pivotTrajectory(torDrive, 180, 1);
        linear3 = new linearTrajectory2(torDrive, 20, 1.5); //1.75 original

    }

    public void run(){
        SmartDashboard.putString("Auto State", AutoState1.toString());
        SmartDashboard.putNumber("Gyro value: ", torDrive.getHeading());
        SmartDashboard.putNumber("Position", torDrive.getPosition());
        switch(AutoState1){
            case INIT:
                linear1.init();
                PrevTime = Timer.getFPGATimestamp();
                AutoState1 = autoRun.LINEAR1;
                
                break;
            case LINEAR1:
                linear1.run();
                mechMaster.autoRun(autoMech.STORE, turretMech.SET, 90);
                if((linear1.isDone())){
                    pivot1.init();
                    torDrive.setMotorSpeeds(0, 0);
                    AutoState1 = autoRun.PIVOT1;
                }
                break;
                
            case PIVOT1:
                pivot1.run();
                mechMaster.autoRun(autoMech.IDLE, turretMech.SET, 90);
                if (pivot1.isDone()) {
                    PrevTime = Timer.getFPGATimestamp();
                    torDrive.setMotorSpeeds(0, 0);
                    AutoState1 = autoRun.SHOOT1;
                }
                break;
                
            case SHOOT1:
                mechMaster.autoRun(autoMech.SHOOT, turretMech.AUTOAIM, 0);
                if ((Timer.getFPGATimestamp() - PrevTime > 3)) {
                    pivot2.init();
                    torDrive.setMotorSpeeds(0, 0);
                    AutoState1 = autoRun.PIVOT2;
                }
                
                break;
            
            case PIVOT2:
                pivot2.run();
                mechMaster.autoRun(autoMech.STORE, turretMech.SET, 0);

                if (pivot2.isDone()) {
                    PrevTime = Timer.getFPGATimestamp();
                    linear2.init();
                    torDrive.setMotorSpeeds(0, 0);
                    AutoState1 = autoRun.LINEAR2;
                }

                break;
            case LINEAR2:
                linear2.run();
                mechMaster.autoRun(autoMech.STORE, turretMech.SET, 0);

                if(Timer.getFPGATimestamp() > PrevTime + 3.0){
                    pivot3.init();
                    PrevTime = Timer.getFPGATimestamp();
                    torDrive.setMotorSpeeds(0, 0);
                    AutoState1 = autoRun.PIVOT3;
                }
                break;
                
            case PIVOT3:
                pivot3.run();
                mechMaster.autoRun(autoMech.IDLE, turretMech.AUTOAIM, 0);
                if (pivot3.isDone()) {
                    torDrive.setMotorSpeeds(0, 0);
                    linear3.init();
                    PrevTime = Timer.getFPGATimestamp();
                    AutoState1 = autoRun.LINEAR3;
                }
                break;

            case LINEAR3:
                linear3.run();
                mechMaster.autoRun(autoMech.SHOOT, turretMech.AUTOAIM, 0);
                if(linear3.isDone()){
                    torDrive.setMotorSpeeds(0, 0);
                    AutoState1 = autoRun.SHOOT2;
                }
                
                break;

            case SHOOT2:
                mechMaster.autoRun(autoMech.SHOOT, turretMech.AUTOAIM, 0);
                if (Timer.getFPGATimestamp() - PrevTime > 4) {
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