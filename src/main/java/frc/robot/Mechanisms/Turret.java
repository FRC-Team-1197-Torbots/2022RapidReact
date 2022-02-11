package frc.robot.Mechanisms;

/*------------------------------------
TURRET CLASS ALLOWS DRIVER TO HOLD DOWN ONE BUTTON TO SHOOT.
CONTROLS 3 THINGS:
    1.LINE UP ROBOT
    2.START/STOP FLYWHEEL
    3.START/STOP ELEVATOR

CALLS METHODS FROM ELEVATOR, FLYWHEEL, AND LIMELIGHTLINEUP CLASS.
------------------------------------*/

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Mechanisms.Elevator.runElevator;

public class Turret {
    public static enum runTurret{
        IDLE, LINEUP, REVUP, SHOOT;
    }
    
    private runTurret turretState = runTurret.IDLE;
    private XboxController player2;
    private LimeLightLineup limelight;
    private Flywheel flywheel;
    private Elevator elevator;


    public Turret(XboxController player2) {
        this.player2 = player2;
        limelight = new LimeLightLineup();
        flywheel = new Flywheel();
        elevator = new Elevator();
    }

    public void run() {
    /*
    CURRENTLY CODING BASED ON ASSUMPTION THAT ROBOT STOPS BEFORE SHOOTING. NOT CONSIDERING MOVING 
    SHOTS JUST YET
    */
        switch(turretState) {
            case IDLE:
                flywheel.run(false, false, 0);
                elevator.run(runElevator.IDLE);

                if (player2.getXButtonPressed()) {
                    turretState = runTurret.LINEUP;
                }
            
            case LINEUP:
                limelight.lineup();
                if (limelight.linedUp()) {
                    flywheel.init();
                    turretState = runTurret.REVUP;
                }
            case REVUP:
                flywheel.run(true, true, limelight.calculateDistance());
                if (flywheel.isFastEnough());
                    //elevator.init();
                    turretState = runTurret.SHOOT;
            case SHOOT:
                elevator.run(runElevator.RUN);
            
            if (player2.getXButtonReleased()) {
                turretState = runTurret.IDLE;
            }
        } 
    }
}
