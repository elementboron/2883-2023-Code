package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimelightReader {

    public static LimelightReader Instance;

    public NetworkTableEntry tx;
    public NetworkTableEntry ty;
    public NetworkTableEntry ta;
    public NetworkTableEntry ts;

    private final double taMax = 1.126;
    private final double maxDist = 128;

    public static void InitLimeLight()
    {
        if(null == Instance)
        {
            Instance = new LimelightReader();
        }
    }


    public void UpdateLimeCam()
    {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        ts = table.getEntry("ts");
    }

    public void UpdateSmartDashboard()
    {
        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double s = ts.getDouble(0.0);
        double area = ta.getDouble(0.0);
    
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightS", s);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("Estimated Cam Distance in Inches", GetEstimatedDistance());
    }

    public double GetEstimatedDistance()
    {
        return maxDist/(((ta.getDouble(0.0)))/(taMax));

    }
}
