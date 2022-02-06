package frc.robot.Mechanisms;

/*------------------------------------
TURRET CLASS ALLOWS DRIVER TO HOLD DOWN ONE BUTTON TO SHOOT.
CONTROLS 3 THINGS:
    1.LINE UP ROBOT
    2.START/STOP FLYWHEEL
    3.START/STOP ELEVATOR

CALLS METHODS FROM ELEVATOR, FLYWHEEL, AND LIMELIGHTLINEUP CLASS.
------------------------------------*/

public class Turret {
    public static enum runTurret{
        IDLE, LINEUP, REVUP, SHOOT, DONE;
    }
    
    private runTurret turretState = runTurret.IDLE;

    public void run() {
        switch(turretState) {
            case IDLE:
                //if button pressed
                    //runState = run.LINEUP
            case LINEUP:
                //if linedup = true
                    //flywheel.init
                    //runState = run.REVUP
            case REVUP:
                //flywheel.revup(limelight.calculateDistance())
                //if flywheel is done
                    //elevator.init
                    //runState = run.SHOOT
            case SHOOT:
                //elevator.run
            case DONE:
                //stop elevator
                //stop shootermotor
            //if button is let go
                //runState = run.DONE
        } 
    }
}
