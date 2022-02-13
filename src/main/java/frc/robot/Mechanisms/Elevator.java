package frc.robot.Mechanisms;
/*----------------------
CONTROLS THE ELEVATOR SPEEDS
----------------------*/

public class Elevator {
    public static enum runElevator {
        IDLE, RUN;
    }

    public void run(runElevator elevatorState) {
        switch(elevatorState) {
            case IDLE:
                //set elevator motor speed to 0
            case RUN:
                //set elevator motor speed to x
        }
    }
    
}
