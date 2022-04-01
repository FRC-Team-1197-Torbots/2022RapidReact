package frc.robot.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*-------------------------
CONTROLS 4 CLIMBER ARMS
-2 BUTTONS TO EXTEND/RETRACT INNER ARMS
-2 BUTTONS TO EXTEND/RETRACT OUTER ARMS
-------------------------*/


public class Climber {

    private CANSparkMax climb;
    private CANSparkMax nikita;

    private RelativeEncoder climbEncoder;
    private RelativeEncoder nikitaEncoder;

    private double speed = 0.9f;
    private double slowSpeed = 0.1;
    private double nikitaSpeed = 0.2;
    public enum climbState{
        UP, DOWN, IDLE, RESET_DOWN;
    }

    public static enum nikitaState {
        UP, DOWN, IDLE;
    }
    
    //public climbState climberPos;

    public Climber(){
        climb = new CANSparkMax(6, MotorType.kBrushless);
        nikita = new CANSparkMax(9, MotorType.kBrushless);

        climbEncoder = climb.getEncoder();
        nikitaEncoder = nikita.getEncoder();

        climbEncoder.setPosition(0);
        nikitaEncoder.setPosition(0);
    }

    public void climb(climbState climberPos){
        //SmartDashboard.putNumber("Right Climber", getRightClimberPos());
        //SmartDashboard.putNumber("Left Climber", getLeftClimberPos());

        switch(climberPos) {
            case UP:
                climb.set(speed);
                break;
            case DOWN:
                if (climbEncoder.getPosition() >= 0)
                    climb.set(-speed);
                else
                    climb.set(0);
                break;
            case RESET_DOWN:
                climb.set(-slowSpeed);
                break;
            case IDLE:
                climb.set(0);
                break;

        }
    }

   /* public double getClimberPos() {
        return climbEncoder.getPosition();
    }

    public boolean isAboveZero() {
        return getClimberPos() > 0;
    }
    */
    public void nikita(nikitaState nikitaPos) {
        switch(nikitaPos) {
            case UP:
                nikita.set(nikitaSpeed);
                break;
            case DOWN:
                nikita.set(-nikitaSpeed);
                break;
            case IDLE:
                nikita.set(0);
                break;
        }
    }

    public double testNikita() {
        return nikitaEncoder.getPosition();
    }
    
    }

