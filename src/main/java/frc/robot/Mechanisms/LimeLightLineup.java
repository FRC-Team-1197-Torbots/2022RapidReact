package frc.robot.Mechanisms;
/*--------------------------------
LIMELIGHTLINEUP CLASS ALIGNS THE TURRET WITH THE CENTER HUB.
ONCE IT'S ALIGNED, IT CAN ALSO CALCULATE DISTANCE.
--------------------------------*/

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimeLightLineup {
        private NetworkTableInstance networkTableInstance;
        private NetworkTable table;// for limelight
        private NetworkTableEntry tx;
        private NetworkTableEntry ty;
        private NetworkTableEntry ta;
        private NetworkTableEntry tv;
        private NetworkTableEntry ts;
        private double x;
        private double y;
        private double area;
        private double v;
        private double s;
        

        public LimeLightLineup(){
                table = NetworkTableInstance.getDefault().getTable("limelight");
                tx = table.getEntry("tx");
                ty = table.getEntry("ty");
                ta = table.getEntry("ta");
                tv = table.getEntry("tv");
                ts = table.getEntry("ts");

                


                
        }
        
        public void init(){

        }

        public void test(){
                
                NetworkTableInstance.getDefault().getTable("limelight");


                tx = table.getEntry("tx");
                ty = table.getEntry("ty");
                ta = table.getEntry("ta");
                tv = table.getEntry("tv");
                ts = table.getEntry("ts");

                // x = tx.getNumber(defaultValue)
                y = ty.getDouble(0.0);
                area = ta.getDouble(0.0);
                v = tv.getDouble(0.5);


               // System.out.println("X: " + x);
                //System.out.println("Y: " + y);
                //System.out.println("Area: " + area);
                
                SmartDashboard.putNumber("LimelightX", tx.getDouble(0.0));
                SmartDashboard.putNumber("LimelightY", ty.getDouble(0.0));
                SmartDashboard.putNumber("LimelightArea", ta.getDouble(0.0));
                SmartDashboard.putNumber("V_Number", tv.getDouble(0.0));
                SmartDashboard.putNumber("S_number", ts.getDouble(0.0));
                
        }

        public double getAngle() {
                x = -tx.getDouble(0.0);
                return x;
        }

        private double degrees_to_radians(double degrees){
                double radians = (degrees * Math.PI) / 180f;
                return radians;
        }

        public double calculate_distance() {
                y = ty.getDouble(0.0);
                double distance = (102.619 - 43.3) / (Math.tan(degrees_to_radians(37) + degrees_to_radians(y)));
                return distance;
                //IF isAligned = true
                //use the function d = (h2-h1) / tan(a1+a2) to calculate
                //h1 = 47.125 inches
                //h2 = 102.619 inches

        }
    
}
