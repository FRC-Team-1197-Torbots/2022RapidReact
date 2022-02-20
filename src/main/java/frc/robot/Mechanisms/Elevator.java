package frc.robot.Mechanisms;
/*----------------------
CONTROLS THE ELEVATOR SPEEDS
----------------------*/

public class Elevator {
    public static enum runElevator {
        IDLE, STORE, SHOOT;
    }

    public void run(runElevator elevatorState) {
        switch(elevatorState) {
            case IDLE:
                //set elevator motor speed to 0
            case STORE:
                //if break beam (digital input) is broken, change state back to idle, else set elevator motor speed to x
            case SHOOT:
                //set elevator motor speed to x
        }
    }
    
}
