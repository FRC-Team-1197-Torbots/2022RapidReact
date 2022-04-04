package frc.robot.Mechanisms;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drive.TorDrive;
import frc.robot.Mechanisms.Climber.climbState;
import frc.robot.Mechanisms.Climber.nikitaState;
import frc.robot.Mechanisms.Elevator.runElevator;
import frc.robot.Mechanisms.Flywheel.runFlywheel;
import frc.robot.Mechanisms.Intake.moveIntake;

/************************************* 
MECHMASTER WILL CONTROL ALL THE MECHANISMS FROM A CENTRAL PLACE
CONTROLS:
    INTAKE
    FLYWHEEL
    LIMELIGHT
    TURRET
    ELEVATOR
    (CLIMBER MAYBE)

TELERUN WILL RUN THE MECHANISMS WHEN ROBOT IS IN TELEOP MODE
AUTORUN WILL RUN THE MECHANISMS WHEN ROBOT IS IN AUTO MODE
***************************************/



public class MechMaster {
    
    private Elevator elevator;
    private Flywheel flywheel;
    private Intake intake;
    private LimeLightLineup limelight;
    private Turret turret;
    private Climber climber;
    private TorDrive drive;


    private XboxController p1;
    private XboxController p2;

    private Intake.moveIntake changeIntake;
    private climbState moveClimber;

    private boolean turretIsAuto = true;

    private boolean climbMode = false;

    private double prevTime;

    public enum autoMech {
        STORE, REVUP, SHOOT, IDLE, SCUFF_SHOOT;
    }

    public enum turretMech {
        SET, AUTOAIM;
    }


    public MechMaster(TorDrive drive) {
        climber = new Climber();
        flywheel = new Flywheel();
        elevator = new Elevator(flywheel);
        intake = new Intake(p1);
        limelight = new LimeLightLineup();
        turret = new Turret();
        p1 = new XboxController(0);
        p2 = new XboxController(1);
        this.drive = drive;
        // intake = new Intake(p1);
        // changeIntake = moveIntake.UP;

        climbMode = false;
    }

    public void TeleInit() {
        elevator.init();
        flywheel.init();
        climbMode = false;
    }

    public void AutoInit() {
        elevator.init();
        flywheel.init();
        Elevator.ballcount = 0;
    }

    public void teleRun() {

        SmartDashboard.putBoolean("Climb mode", climbMode);
        SmartDashboard.putBoolean("Turret mode", turretIsAuto);
        //System.out.println("Climb mode: " + climbMode);
        //System.out.println("A button: " + p2.getAButtonPressed());

        //TOGGLE CLIMB MODE
        if (p2.getAButtonPressed()) {
            climbMode = !climbMode;
        }

        if (climbMode) {
            //CLIMBER
            elevator.run(runElevator.OFF);
            flywheel.run(runFlywheel.OFF, 0);
            turret.PIDTuning(0);
            intake.run(moveIntake.OFF);
            

            if(p2.getPOV() == 0 ){
                climber.climb(climbState.UP);  
            }
            else if(p2.getPOV() == 180){ //&& climber.isAboveZero()){
                climber.climb(climbState.DOWN);
            }
            else {
                climber.climb(climbState.IDLE);
            }
        
                

            
            if (p2.getPOV() == 90) {
                climber.nikita(nikitaState.UP);
            }
            else if (p2.getPOV() == 270) {
                climber.nikita(nikitaState.DOWN);
            }
            else
                climber.nikita(nikitaState.IDLE);
            }
        else {

        
            //TURRET CONTROL
            if (p2.getYButtonPressed()) {
                turretIsAuto = !turretIsAuto;
            }
            if (turretIsAuto) {
                turret.PIDTuning(limelight.getAngle());
            }
            else {
                if (p2.getBButton())
                    turret.PIDTuning(90);
                else if (p2.getXButton())
                    turret.PIDTuning(-90);
                //else if (p2.getAButton())
                   // turret.PIDTuning(0);
                else
                    turret.manualControl(0f);
            }

            //SmartDashboard.putBoolean("Ball in elevator: ", elevator.isBallInElevator());
            //maybe override the intake w the A button
            //SmartDashboard.putBoolean("LeftBumper Pressed: ", p1.getLeftBumper());

            //SHOOTING
            
            if (p1.getRightTriggerAxis() >= 0.95) {
                //System.out.println("Limelight distance: " + limelight.calculate_distance());

                flywheel.run(runFlywheel.RUN, limelight.calculate_distance());
                
                
                if (p1.getLeftBumper() == true 
                    && flywheel.OnTarget
                    && Math.abs(drive.getRightVelocity()) < 1000 
                    && Math.abs(drive.getLeftVelocity()) < 1000) 
                {
                    elevator.run(runElevator.SHOOT);
                }
                else
                    elevator.run(runElevator.STORE);
                

                /*
                if ( flywheel.OnTarget
                    && Math.abs(drive.getRightVelocity()) < 1000 
                    && Math.abs(drive.getLeftVelocity()) < 1000) 
                {
                    elevator.run(runElevator.SHOOT);
                }
                else {
                    elevator.run(runElevator.STORE);
                    prevTime = Timer.getFPGATimestamp();
                    
                }
                */
                
            }

            //TEST SHOOTING
            /*
            if (p1.getRightTriggerAxis() >= 0.95) {
                flywheel.testRun(-2200);
                //System.out.println("Limelight distance: " + limelight.calculate_distance());
                //SmartDashboard.putNumber("Limerlight Distance", limelight.calculate_distance());

                if (p1.getLeftBumper() == true 
                    && flywheel.OnTarget
                    && Math.abs(drive.getRightVelocity()) < 1000 
                    && Math.abs(drive.getLeftVelocity()) < 1000) 
                {
                    elevator.run(runElevator.SHOOT);
                }
                else
                    elevator.run(runElevator.STORE);
            }
            */

            //INTAKING
            else if (p1.getAButton()) {
                if(elevator.ballcount < 2) {
                    intake.run(moveIntake.DOWN);
                    elevator.run(runElevator.STORE);
                } else {
                    intake.run(moveIntake.UP);
                    elevator.run(runElevator.IDLE);
                }
            }
            else {
                intake.run(moveIntake.UP);
                elevator.run(runElevator.IDLE);
                flywheel.run(runFlywheel.IDLE, 0);
            }


            //RESETS THE BALL COUNT
            if (p2.getLeftTriggerAxis() >= 0.95) {
                elevator.resetBallCount();
                flywheel.setPIDValues(1);
            }

            //RESETS PID VALUES OF THE FLYWHEEL WHEN TRIGGER LET GO
            if (p1.getRightTriggerAxis() <= 0.05) {
                flywheel.setPIDValues(1);
            }
        }

            
            
            

            //TEST NIKITA
            //System.out.println("Nikita power: " + climber.testNikita());
            
            
            
            
            //TESTING DIFFERENT DISTANCES FOR FLYWHEEL
            //(10 feet, 2200 rpm)
            //
            /*
            elevator.run(runElevator.SHOOT);
            flywheel.testRun(-1800f);
            limelight.test();
            SmartDashboard.putNumber("Limelight distance", limelight.calculate_distance());
            //System.out.println("Distance: " + limelight.calculate_distance());
            //turret.PIDTuning(limelight.getAngle());
            
            /*
            //TUNING THE PID FOR FLYWHEEL
            //elevator.run(runElevator.SHOOT);
            flywheel.testRun(-2000);

            
            /*
            if (p2.getPOV(90) == 90){
                turret.manualControl("right");
            }
            else if (p2.getPOV(270) == 270){
                turret.manualControl("left");
            }
            else{
                //turret.PIDTuning(tx);
            }
            */    
    }

    public void autoRun(autoMech mechState, turretMech turretState, double angle) {
        climbMode = false;

        switch(turretState) {
            case SET:
                turret.PIDAutoTuning(angle);
                break;
            case AUTOAIM:
                turret.PIDTuning(limelight.getAngle());
                break;
        }

        switch(mechState) {
            case STORE:
                intake.run(moveIntake.DOWN);
                elevator.run(runElevator.STORE);
                flywheel.run(runFlywheel.RUN, limelight.calculate_distance());

                break;
            case REVUP:
                intake.run(moveIntake.UP);
                flywheel.testRun(-2640); //-2590
                elevator.run(runElevator.STORE);
                break;
            case SHOOT:
                //if (Math.abs(limelight.getAngle()) < 1)
                intake.run(moveIntake.UP);
                flywheel.run(runFlywheel.RUN, limelight.calculate_distance());
                if(flywheel.OnTarget
                    && Math.abs(drive.getRightVelocity()) < 1000 
                    && Math.abs(drive.getLeftVelocity()) < 1000)
                {
                    elevator.run(runElevator.SHOOT);
                }
                else
                    elevator.run(runElevator.STORE);
                /*
                if(flywheel.OnTarget) {
                    elevator.run(runElevator.SHOOT);
                    System.out.println("Shooting");
                } else {
                    elevator.run(runElevator.IDLE);
                    System.out.println("Idle");
                }
                */
                
                break;
            
            case SCUFF_SHOOT:
                intake.run(moveIntake.UP);
                flywheel.testRun(-2640); //-2590
                if(flywheel.OnTarget)
                {
                    elevator.run(runElevator.SHOOT);
                }
                else
                    elevator.run(runElevator.STORE);
                break;

            
            case IDLE:
                intake.run(moveIntake.UP);
                elevator.run(runElevator.IDLE);
                flywheel.run(runFlywheel.RUN, limelight.calculate_distance());
                break;
        }

    }
    
    public int getBallCount() {
        return elevator.ballcount;
    }

    public void onDisable(){
        flywheel.onDisable();
        turret.onDisable();
    }



}
