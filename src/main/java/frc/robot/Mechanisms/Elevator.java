package frc.robot.Mechanisms;
/*----------------------
CONTROLS THE ELEVATOR SPEEDS
----------------------*/

import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;

public class Elevator {

    private DigitalInput breakbeam;
    private CANSparkMax elMotor;
    private Timer timer;

    private boolean ballInElevator = false;

    public Elevator() {
        breakbeam = new DigitalInput(0);
        elMotor = new CANSparkMax(10, MotorType.kBrushless);
        timer = new Timer();
        timer.reset();
    }


    public static enum runElevator {
        IDLE, STORE, SHOOT;
    }

    public void run(runElevator elevatorState) {
        switch(elevatorState) {
            case IDLE:
                elMotor.set(0);
                //set elevator motor speed to 0
            break;
            case STORE:
                //if break beam (digital input) is broken, set motor speed to 0, else set elevator motor speed to x
                if (!breakbeam.get()){ //or !breakbeam.get()
                    ballInElevator = true;
                    elMotor.set(0);
                }
                else
                    elMotor.set(0.6);
            break;
            case SHOOT:
                //set elevator motor speed to x
                elMotor.set(0.6);
                ballInElevator = false;
            break;
        }
    }

    public void testBreakbeam() {
        System.out.println("Breakbeam: " + breakbeam.get());
        
    }
    
    public boolean isBallInElevator() {
        return ballInElevator;
    }
}