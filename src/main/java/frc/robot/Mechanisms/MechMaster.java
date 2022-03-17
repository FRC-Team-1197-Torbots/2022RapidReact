package frc.robot.Mechanisms;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Mechanisms.Climber.climbState;
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

    private XboxController p1;
    private XboxController p2;

    private Intake.moveIntake changeIntake;
    private climbState moveClimber;

    private boolean turretIsAuto = true;

    public enum autoMech {
        STORE, SHOOT, IDLE;
    }


    public MechMaster() {
        climber = new Climber();
        flywheel = new Flywheel();
        elevator = new Elevator(flywheel);
        intake = new Intake(p1);
        limelight = new LimeLightLineup();
        turret = new Turret();
        p1 = new XboxController(0);
        p2 = new XboxController(1);
        // intake = new Intake(p1);
        // changeIntake = moveIntake.UP;
    }

    public void TeleInit() {
        elevator.init();
        flywheel.init();
    }

    public void AutoInit() {
        elevator.init();
        flywheel.init();
        Elevator.ballcount = 1;
    }

    public void teleRun() {
        
        
        /*
        if(p1.getXButtonPressed() && intake.ONTARGET){
            intake.run(true);        
            System.out.println("Button pressed: "+ p1.getXButtonPressed());
        } else {
            intake.run(false);
            System.out.println("Button pressed: "+ p1.getXButtonPressed());
        }
        */
        

        //testing if intake sensor is working
        /*
        if (p1.getAButton()) {
            intake.testrun(1);
        }
        else if (p1.getYButton()) {
            intake.testrun(2);
        }
        SmartDashboard.getNumber("intake encoder", intake.getEncoderPos());
        */
        
        //AFTER INTAKE IS TUNED, RUN IT WITH THE ELEVATOR LOGIC (COMMENT OUT THE FLYWHEEL CLASS)
        SmartDashboard.putBoolean("turretIsAuto?", turretIsAuto);
        
        if (p2.getYButtonPressed()) {
            turretIsAuto = !turretIsAuto;
        }
        
        if (turretIsAuto) {
            turret.PIDTuning(limelight.getAngle());
        }
        else {
            if (p2.getBButton())
                turret.manualControl(-0.2f);
            else if (p2.getXButton())
                turret.manualControl(0.2f);
            else if (p2.getAButton())
                turret.manualZero();
            else
                turret.manualControl(0f);
        }
        
        
        
        SmartDashboard.putBoolean("Ball in elevator: ", elevator.isBallInElevator());
        if (p1.getAButton() && p2.getRightTriggerAxis() == 1) {
            intake.run(moveIntake.DOWN);
            elevator.run(runElevator.SHOOT);
            flywheel.run(runFlywheel.RUN, limelight.calculate_distance());
        }

        else if (p1.getAButton()) {
            if(elevator.ballcount < 2) {
                intake.run(moveIntake.DOWN);
                elevator.run(runElevator.STORE);
            } else {
                intake.run(moveIntake.UP);
                elevator.run(runElevator.IDLE);
            }
                
           // if (!elevator.isBallInElevator())
             //   elevator.run(runElevator.STORE);
            //else
            
            //flywheel.run(runFlywheel.IDLE, 0);
        }

        else if (p2.getRightTriggerAxis() == 1) {
            elevator.run(runElevator.SHOOT);
            flywheel.run(runFlywheel.RUN, limelight.calculate_distance());
        }
        else {
            intake.run(moveIntake.UP);
            elevator.run(runElevator.IDLE);
            flywheel.run(runFlywheel.IDLE, 0);
        }

        if (p2.getLeftBumperPressed()) {
            elevator.resetBallCount();
            flywheel.setPIDValues(1);
        }
        

        //CLIMBER
        

        if(p2.getPOV() == 0){
            climber.climb(climbState.UP);  
        }
        else if(p2.getPOV() == 180){
            climber.climb(climbState.DOWN);
        }
        else
            climber.climb(climbState.IDLE);

        //NIKITA
        /*
        if (p2.getPOV() == 90) {
            climber.nikita(nikitaState.UP);
        }
        else if (p2.getPOV() == 270) {
            climber.nikita(nikitaState.DOWN);
        }
        else
            climber.nikita(nikitaState.IDLE);
        
        */

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

    public void autoRun(autoMech mechState) {
        turret.PIDTuning(limelight.getAngle());

        switch(mechState) {
            case STORE:
                intake.run(moveIntake.DOWN);
                elevator.run(runElevator.STORE);
                flywheel.run(runFlywheel.IDLE, 0);
                break;
            case SHOOT:
                flywheel.run(runFlywheel.RUN, limelight.calculate_distance());
                elevator.run(runElevator.SHOOT);
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
            
            case IDLE:
                intake.run(moveIntake.UP);
                elevator.run(runElevator.IDLE);
                flywheel.run(runFlywheel.IDLE, 0);
                break;
        }

    }
    
    public int getBallCount() {
        return elevator.ballcount;
    }

    public void onDisable(){
        flywheel.onDisable();
    }



}
