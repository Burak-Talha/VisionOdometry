package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class Constants {
    public static final List<AprilTag> aprilTags =
        List.of(
          new AprilTag(1,
              new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(42.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, Math.PI))),
              
          new AprilTag(2,
            new Pose3d(
                Units.inchesToMeters(610.77),
                Units.inchesToMeters(108.19),
                Units.inchesToMeters(18.22),
                new Rotation3d(0.0, 0.0, Math.PI))),

          new AprilTag(3,
            new Pose3d(
                Units.inchesToMeters(610.77),
                Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                Units.inchesToMeters(18.22),
                new Rotation3d(0.0, 0.0, Math.PI))),

          new AprilTag(4,
            new Pose3d(
                Units.inchesToMeters(636.96),
                Units.inchesToMeters(265.74),
                Units.inchesToMeters(27.38),
                new Rotation3d(0.0, 0.0, Math.PI))),

          new AprilTag(5,
            new Pose3d(
                Units.inchesToMeters(14.25),
                Units.inchesToMeters(265.74),
                Units.inchesToMeters(27.38),
                new Rotation3d())),

          new AprilTag(6,
            new Pose3d(
                Units.inchesToMeters(40.45),
                Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                Units.inchesToMeters(18.22),
                new Rotation3d())),

          new AprilTag(7,
          new Pose3d(
              Units.inchesToMeters(40.45),
              Units.inchesToMeters(108.19),
              Units.inchesToMeters(18.22),
              new Rotation3d())),

          new AprilTag(8,
          new Pose3d(
              Units.inchesToMeters(40.45),
              Units.inchesToMeters(42.19),
              Units.inchesToMeters(18.22),
              new Rotation3d()))
      );

      public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = new AprilTagFieldLayout(aprilTags, 16.5, 8);
}
