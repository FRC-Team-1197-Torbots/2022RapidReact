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
    private DigitalInput ShooterBeam;
    private CANSparkMax elMotor;

    private boolean ballInElevator = false;
    public static int ballcount;
    private boolean prev, shooterprev;
    private double PrevTime;

    private Flywheel flywheel;

    public Elevator(Flywheel flywheel) {
        this.flywheel = flywheel;
        breakbeam = new DigitalInput(0);
        ShooterBeam = new DigitalInput(2);
        elMotor = new CANSparkMax(10, MotorType.kBrushless);
        prev = breakbeam.get();
        shooterprev = ShooterBeam.get();
        ballcount = 0;
    }

    public void init() {
        PrevTime = Timer.getFPGATimestamp() + 0.2f;
    }


    public static enum runElevator {
        IDLE, STORE, SHOOT;
    }

    public static enum autoElevator {
        IDLE, STORE, SHOOT;
    }

    public void run(runElevator elevatorState) {
        testBreakbeam();

        switch(elevatorState) {
            case IDLE:                
                if(ballcount < 2) {    
                    elMotor.set(0.15f);

                    if (breakbeam.get() && !prev){ 
                        ballcount++;                        
                    }

                    prev = breakbeam.get();                    
                } else if(ballcount == 2) {
                    elMotor.set(0);
                }

                PrevTime = Timer.getFPGATimestamp();                                              
            break;

            case STORE:
                if(ballcount < 2) {                    
                    elMotor.set(0.4);
                    if (breakbeam.get() && !prev){ 
                        ballcount++;                        
                    }
                    prev = breakbeam.get();                    
                }
                else if (ballcount == 2) {
                    elMotor.set(0);
                }

                
                PrevTime = Timer.getFPGATimestamp();                
            break;

            case SHOOT:                
                    if(Timer.getFPGATimestamp() < PrevTime + 0.15f && !flywheel.OnTarget) {                    
                        elMotor.set(-0.6);                    
                    } else if (flywheel.OnTarget) {
                        elMotor.set(0.6);
                    
                        if(ShooterBeam.get() && !shooterprev) {
                            ballcount--;
    
                            if (ballcount < 0)
                                ballcount = 0;
    
                            // SWITCHES PID VALUES
                            if (ballcount == 1) {
                                flywheel.setPIDValues(2);
                            }
                            else if(ballcount == 0) {
                                flywheel.pidIntegral = 0;
                                flywheel.setPIDValues(1);
                            }
    
                        }                   
                            
                        shooterprev = ShooterBeam.get();
                    } else if(!flywheel.OnTarget) {
                        elMotor.set(0);
                    }    
                           
            break;
            
        }
    }
    /*
    public void autoRun(autoElevator autoElState) {
        switch (autoElState) {
            case IDLE: 
                if(ballcount < 2) {    
                    elMotor.set(0.2f);

                    if (breakbeam.get() && !prev){ 
                        ballcount++;                        
                    }

                    prev = breakbeam.get();                    
                } else if(ballcount == 2) {
                    elMotor.set(0);
                }

                PrevTime = Timer.getFPGATimestamp();
        }
    }
    */


    public void testBreakbeam() {
        SmartDashboard.putBoolean("Breakbeam", breakbeam.get()); 
        SmartDashboard.putBoolean("Shooter Beam", ShooterBeam.get());
        SmartDashboard.putNumber("Balls in robot", ballcount);       
    }
    
    public boolean isBallInElevator() {
        return ballInElevator;
    }

    public void resetBallCount() {
        ballcount = 0;
    }

    public void setBallCount(int count) {
        ballcount = count;
    }
}