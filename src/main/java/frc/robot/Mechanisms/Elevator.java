package frc.robot.Mechanisms;
/*----------------------
CONTROLS THE ELEVATOR SPEEDS
----------------------*/

import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {

    private DigitalInput breakbeam;
    private CANSparkMax elMotor;
    private Timer timer;

    private boolean ballInElevator = false;
    public int ballcount;
    private boolean prev;

    public Elevator() {
        breakbeam = new DigitalInput(0);
        elMotor = new CANSparkMax(10, MotorType.kBrushless);
        timer = new Timer();
        timer.reset();

        prev = false;
    }


    public static enum runElevator {
        IDLE, STORE, SHOOT, DOWN;
    }

    public void run(runElevator elevatorState) {
        testBreakbeam();

        switch(elevatorState) {
            case IDLE:
                elMotor.set(0);
                //set elevator motor speed to 0
            break;
            case STORE:
                if(ballcount < 2) {
                    //if break beam (digital input) is broken, set motor speed to 0, else set elevator motor speed to x
                    if (!breakbeam.get() && !prev){ 
                        ballcount++;                        
                    }

                    prev = !breakbeam.get();
                    
                } 

                elMotor.set(0.4);                
            break;
            case SHOOT:
                //set elevator motor speed to x
                elMotor.set(0.6);
                ballInElevator = false;
            break;
            case DOWN:
                if(breakbeam.get()) {
                    elMotor.set(-0.6);
                } else {
                    elMotor.set(0);
                }
                
            break;
        }
    }

    public void testBreakbeam() {
        SmartDashboard.putBoolean("Breakbeam", breakbeam.get()); 
        SmartDashboard.putNumber("Balls in robot", ballcount);       
    }
    
    public boolean isBallInElevator() {
        return ballInElevator;
    }
}