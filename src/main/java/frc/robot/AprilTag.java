package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AprilTag
{
    int ID = 0;
    String name ="";
    Pose2d Pose = new Pose2d(0, 0, new Rotation2d(0));
    double yRotatinDegrees = 0;
    boolean algaeOnUpper = false;
    double extraOffsetWhenTargeting = 0;//(+) away from tag (-) towards the tag (from robot view)
    double offset90Offset = 0;//Positive number will spread left and right further from center tag point.
    public AprilTag(int _ID,String _name, Pose2d _position,  double _yRot)
    {
      ID = _ID;
      name =_name;
      Pose = _position;
      yRotatinDegrees = _yRot;
    }

    public AprilTag Withoffset90(double extraOffset)
    {
      offset90Offset = extraOffset;
      return this;
    }
    public AprilTag Withdepthoffset(double extraOffset)
    {
      extraOffsetWhenTargeting = extraOffset;
      return this;
    }
    public AprilTag WithAlgaeOnUpper()
    {
      algaeOnUpper = true;
      return this;
    }

}