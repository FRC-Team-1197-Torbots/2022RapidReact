package frc.robot.Mechanisms;
/*--------------------------------
LIMELIGHTLINEUP CLASS ALIGNS THE TURRET WITH THE CENTER HUB.
ONCE IT'S ALIGNED, IT CAN ALSO CALCULATE DISTANCE.
--------------------------------*/


public class LimeLightLineup {
    public void init(){

    }

    public boolean linedUp() {
            return false;
    }

    public void lineup(){
        /*
        float Kp = -0.1f;
        float min_command = 0.05f;

        std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
        float tx = table->GetNumber("tx");

        if (joystick->GetRawButton(9))
        {
                float heading_error = -tx;
                float steering_adjust = 0.0f;
                if (tx > 1.0)
                {
                        steering_adjust = Kp*heading_error - min_command;
                }
                else if (tx < 1.0)
                {
                        steering_adjust = Kp*heading_error + min_command;
                }
                left_command += steering_adjust;
                right_command -= steering_adjust;
        }
        */
    }
    public double calculateDistance(){
        return 0.0;
        //IF isAligned = true
            //use the function d = (h2-h1) / tan(a1+a2) to calculate
    }

    
}
