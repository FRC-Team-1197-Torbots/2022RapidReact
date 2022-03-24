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

    private DigitalInput lowBeam;
    private DigitalInput ShooterBeam;
    private CANSparkMax elMotor;

    private boolean ballInElevator = false;
    public static int ballcount;
    private boolean prev, shooterprev;
    private double PrevTime;

    private Flywheel flywheel;

    public Elevator(Flywheel flywheel) {
        this.flywheel = flywheel;
        lowBeam = new DigitalInput(0);
        ShooterBeam = new DigitalInput(2);
        elMotor = new CANSparkMax(10, MotorType.kBrushless);
        prev = lowBeam.get();
        shooterprev = ShooterBeam.get();
        ballcount = 0;
    }

    public void init() {
        PrevTime = Timer.getFPGATimestamp() + 0.2f;
    }


    public static enum runElevator {
        IDLE, STORE, SHOOT, REVUP;
    }

    public static enum autoElevator {
        IDLE, STORE, SHOOT;
    }

    public void run(runElevator elevatorState) {
        testBreakBeam();

        switch(elevatorState) {
            case IDLE:                
                if(ballcount < 2) {    
                    elMotor.set(0.4f); //0.15

                    if (lowBeam.get() && !prev){ 
                        ballcount++;                        
                    }

                    prev = lowBeam.get();                    
                } else if(ballcount == 2) {
                    elMotor.set(0);
                }

                PrevTime = Timer.getFPGATimestamp();
                
                /*
                if (ballcount == 0) {
                    elMotorTop.set(0.8);
                    elMotorBottom.set(0.8);
                    if (topBeam.get() && !prevTopBeam) {
                        ballcount++
                    }
                }
                else if (ballcount == 1) {
                    elMotorTop.set(0);
                    elMotorBottom.set(0.8);
                    if (bottomBeam.get() && !prevBottomBeam) {
                        ballcount++;
                    }
                }
                else if (ballcount == 2) {
                    elMotorTop.set(0);
                    elMotorBottom.set(0);
                }

                prevTopBeam = topBeam;
                prevBottomBeam = bottomBeam;
                */
            break;

            case STORE:
                if(ballcount < 2) {                    
                    elMotor.set(0.4);
                    if (lowBeam.get() && !prev){ 
                        ballcount++;                        
                    }
                    prev = lowBeam.get();                    
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

                    /*
                    if (flywheel.OnTarget) {
                        elMotor.set(0.6);
                        if (ballcount == 2) {
                            if (!topBeam.get() && prevTopBeam.get())
                                ballcount--;
                        }
                        else if (ballcount == 1) {
                            if (!topBeam.get() && prevTopBeam.get())
                                ballcount--;
                        }
                        else if (ballcount == 0) {
                            nuffin...
                        }
                        
                    }
                    else
                        elMotor.set(0);

                    prevBottomBeam = bottomBeam;
                    prevTopBeam = topBeam;

                    */
                           
            break;

            case REVUP:
                if(Timer.getFPGATimestamp() < PrevTime + 0.15f && !flywheel.OnTarget) {                    
                    elMotor.set(-0.6);                    
                } else {
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

                    if (lowBeam.get() && !prev){ 
                        ballcount++;                        
                    }

                    prev = lowBeam.get();                    
                } else if(ballcount == 2) {
                    elMotor.set(0);
                }

                PrevTime = Timer.getFPGATimestamp();
        }
    }
    */


    public void testBreakBeam() {
        SmartDashboard.putBoolean("lowBeam", lowBeam.get()); 
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