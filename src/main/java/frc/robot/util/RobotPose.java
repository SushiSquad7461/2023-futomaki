package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotPose {
   public Vector loc;
   public Rotation2d rot;

   public RobotPose(Vector loc, Rotation2d rot) {
    this.loc = loc;
    this.rot = rot;
   }

   public Pose2d getPose2d() {
      return new Pose2d(
         new Translation2d(loc.x, loc.y),
         rot
      );
   }
}
