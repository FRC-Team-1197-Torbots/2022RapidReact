package frc.robot.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/*-------------------------
CONTROLS 4 CLIMBER ARMS
-2 BUTTONS TO EXTEND/RETRACT INNER ARMS
-2 BUTTONS TO EXTEND/RETRACT OUTER ARMS
-------------------------*/


public class Climber {
    private MechMaster mechMaster;

    private CANSparkMax rightClMotor;
    private CANSparkMax leftClMotor;
    private CANSparkMax nikitaClMotor;

    private double speed = 0.2;
    private double nikitaSpeed = 0.2;

    public enum climbState{
        UP, DOWN;
    }

    public enum nikitaState {
        UP, DOWN;
    }
    
    //public climbState climberPos;

    public Climber(){
        mechMaster = new MechMaster();
        rightClMotor = new CANSparkMax(9, MotorType.kBrushless);
        leftClMotor = new CANSparkMax(6, MotorType.kBrushless);
        nikitaClMotor = new CANSparkMax(3, MotorType.kBrushless);
        


    }

    public void climb(climbState climberPos){
        switch(climberPos) {
            case UP:
                rightClMotor.set(speed);
                leftClMotor.set(-speed);
                break;
            case DOWN:
                rightClMotor.set(-speed);
                leftClMotor.set(speed);
                break;

        }
    }

    public void nikita(nikitaState nikitaPos) {
        switch(nikitaPos) {
            case UP:
                nikitaClMotor.set(nikitaSpeed);
            case DOWN:
                nikitaClMotor.set(-nikitaSpeed);
        }
    }

    
}
