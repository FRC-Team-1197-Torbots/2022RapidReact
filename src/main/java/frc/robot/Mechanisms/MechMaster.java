package frc.robot.Mechanisms;

import edu.wpi.first.wpilibj.XboxController;
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

    private XboxController p1;
    private XboxController p2;

    private Intake.moveIntake changeIntake;


    public MechMaster() {
        //elevator = new Elevator();
        // flywheel = new Flywheel();
        //intake = new Intake(p1);
        // limelight = new LimeLightLineup();
        // turret = new Turret();
        p1 = new XboxController(0);
        p2 = new XboxController(1);
        intake = new Intake(p1);
        changeIntake = moveIntake.UP;
    }

    public void teleRun() {
        if(p1.getXButtonPressed() && intake.ONTARGET){
            intake.run(true);        
        } else {
            intake.run(false);
        }
        
        //AFTER INTAKE IS TUNED, RUN IT WITH THE ELEVATOR LOGIC (COMMENT OUT THE FLYWHEEL CLASS)
        /*

        if (p1.getAButton() && p2.getRightTriggerAxis() == 1) {
            intake.run(moveIntake.DOWN);
            elevator.run(runElevator.SHOOT);
            flywheel.run(runFlywheel.RUN, limelight.calculate_distance());
        }

        else if (p1.getAButton()) {
            intake.run(moveIntake.DOWN);
            if (!elevator.isBallInElevator())
                elevator.run(runElevator.STORE);
            else
                elevator.run(runElevator.IDLE);
            flywheel.run(runFlywheel.IDLE, 0);
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

        

        if (p2.getPov(90)){
            turret.manualControl("right");
        }
        else if (p2.getPov(270)){
            turret.manualControl("left");
        }
        else{
            turret.PIDTuning(tx);
        }
    */



    
    }

    public void autoRun() {

    }



}
