package frc.robot.Mechanisms;

import edu.wpi.first.wpilibj.XboxController;

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
    private XboxController player2;


    public MechMaster() {
        elevator = new Elevator();
        flywheel = new Flywheel();
        intake = new Intake();
        limelight = new LimeLightLineup();
        turret = new Turret();
        player2 = new XboxController(1);

    }

    public void teleRun() {
        
        /*
        if (p1.getabutton() && p2.gettriggerbutton()) {
            intake.run(DOWN);
            elevator.run(SHOOT);
            flywheel.run(runFlywheel.RUN, limelight.calculateDistance())
        }

        else if (p1.getAButton()) {
            intake.run(DOWN);
            if (!elevator.isballinelevator)
                elevator.run(STORE);
            else
                elevator.run(IDLE);
            flywheel.run(runFlywheel.IDLE)
        }

        else if (p2.gettriggerbutton()) {
            elevator.run(SHOOT);
            flywheel.run(runFlywheel.RUN, limelight.calculateDistance())
        }
        else {
            intake.run(UP);
            elevator.run(IDLE);
            flywhell.run(runFlywheel.IDLE);
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
