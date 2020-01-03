package net.teamrush27.frc2019.base;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
import net.teamrush27.frc2019.constants.TrackingConstants;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.util.interpolate.InterpolatingDouble;
import net.teamrush27.frc2019.util.interpolate.InterpolatingTreeMap;
import net.teamrush27.frc2019.util.math.KinematicsUtils;
import net.teamrush27.frc2019.util.math.MathUtils;
import net.teamrush27.frc2019.util.math.Pose2d;
import net.teamrush27.frc2019.util.math.Rotation2d;
import net.teamrush27.frc2019.util.math.Translation2d;
import net.teamrush27.frc2019.util.math.Twist2d;

public class RobotState {

  private static RobotState instance_ = new RobotState();

  public static RobotState getInstance() {
    return instance_;
  }

  private static final int kObservationBufferSize = 100;

  // FPGATimestamp -> RigidTransform2d or Rotation2d
  private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
  private Twist2d vehicle_velocity_predicted_;
  private Twist2d vehicle_velocity_measured_;
  private double distance_driven_;

  private RobotState() {
    reset(0, new Pose2d());
  }

  /** Resets the field to robot transform (robot's position on the field) */
  public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
    field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
    field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
    Drivetrain.getInstance().setHeading(initial_field_to_vehicle.getRotation());
    vehicle_velocity_predicted_ = Twist2d.identity();
    vehicle_velocity_measured_ = Twist2d.identity();
    distance_driven_ = 0.0;
  }

  public synchronized void resetDistanceDriven() {
    distance_driven_ = 0.0;
  }

  /**
   * Returns the robot's position on the field at a certain time. Linearly interpolates between
   * stored robot positions to fill in the gaps.
   */
  public synchronized Pose2d getFieldToVehicle(double timestamp) {
    return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
  }

  public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
    return field_to_vehicle_.lastEntry();
  }

  public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
    return getLatestFieldToVehicle()
        .getValue()
        .transformBy(Pose2d.exp(vehicle_velocity_predicted_.scaled(lookahead_time)));
  }

  public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
    field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
  }

  public synchronized void addObservations(
      double timestamp, Twist2d measured_velocity, Twist2d predicted_velocity) {
    addFieldToVehicleObservation(
        timestamp,
        KinematicsUtils.integrateForwardKinematics(
            getLatestFieldToVehicle().getValue(), measured_velocity));
    vehicle_velocity_measured_ = measured_velocity;
    vehicle_velocity_predicted_ = predicted_velocity;
  }

  public synchronized Twist2d generateOdometryFromSensors(
      double left_encoder_delta_distance,
      double right_encoder_delta_distance,
      Rotation2d current_gyro_angle) {
    final Pose2d last_measurement = getLatestFieldToVehicle().getValue();
    final Twist2d delta =
        KinematicsUtils.forwardKinematics(
            last_measurement.getRotation(),
            left_encoder_delta_distance,
            right_encoder_delta_distance,
            current_gyro_angle);
    distance_driven_ += delta.deltaX; // do we care about dy here?
    return delta;
  }

  public synchronized double getDistanceDriven() {
    return distance_driven_;
  }

  public synchronized Twist2d getPredictedVelocity() {
    return vehicle_velocity_predicted_;
  }

  public synchronized Twist2d getMeasuredVelocity() {
    return vehicle_velocity_measured_;
  }

  public void outputToSmartDashboard() {
    Pose2d odometry = getLatestFieldToVehicle().getValue();
    SmartDashboard.putNumber("Robot Pose X", odometry.getTranslation().x());
    SmartDashboard.putNumber("Robot Pose Y", odometry.getTranslation().y());
    SmartDashboard.putNumber("Robot Pose Theta", odometry.getRotation().getDegrees());
    SmartDashboard.putNumber("Robot Linear Velocity", vehicle_velocity_measured_.deltaX);
  }

  public enum Target {
    SHIP_FRONT(new Pose2d(219, 152, Rotation2d.identity())),
    SHIP_SIDE_1(new Pose2d(260, 136, Rotation2d.fromDegrees(90))),
    SHIP_SIDE_2(new Pose2d(282, 136, Rotation2d.fromDegrees(90))),
    SHIP_SIDE_3(new Pose2d(303, 136, Rotation2d.fromDegrees(90))),
    ROCKET_CLOSE_SIDE(new Pose2d(214, 20, Rotation2d.fromDegrees(-28.75))),
    ROCKET_CARGO_SIDE(new Pose2d(229, 29, Rotation2d.fromDegrees(-90))),
    ROCKET_FAR_SIDE(new Pose2d(243, 20, Rotation2d.fromDegrees(-151.75))),
    HUMAN_PLAYER_STATION(new Pose2d(0, 28, Rotation2d.fromDegrees(-180))),
    NO_TARGET(
        new Pose2d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Rotation2d.identity()));

    private static final Target[] TARGET_ARR =
        new Target[] {
          SHIP_FRONT,
          SHIP_SIDE_1,
          SHIP_SIDE_2,
          SHIP_SIDE_3,
          ROCKET_CLOSE_SIDE,
          ROCKET_CARGO_SIDE,
          ROCKET_FAR_SIDE,
          HUMAN_PLAYER_STATION
        };

    private final Pose2d targetOrientation;

    Target(Pose2d orientation) {
      this.targetOrientation = orientation;
    }

    public Pose2d getTargetOrientation() {
      return this.targetOrientation;
    }
  }

  // Don't worry about the orientation of the target relative to our orientation,
  // that's just a matter of ending up same or opposite direction when we get there
  private Optional<Target> getNearestFacedTargetInternal(Pose2d location) {
    return Arrays.stream(Target.TARGET_ARR)
        .filter(
            t ->
                Math.abs(location.rotationTo(t.getTargetOrientation()).getRadians())
                        < TrackingConstants.HALF_FOV
                    && location.distance(t.getTargetOrientation()) < TrackingConstants.FOV_RADIUS)
        .sorted(
            (t1, t2) -> {
              double d1 = location.distance(t1.getTargetOrientation());
              double d2 = location.distance(t2.getTargetOrientation());
              if (MathUtils.epsilonEquals(d1, d2, 0.5)) {
                return 0;
              }
              return d1 > d2 ? 1 : -1;
            })
        .findAny();
  }

  public Target getNearestFacedTarget(boolean reverse) {
    Pose2d location = RobotState.getInstance().getLatestFieldToVehicle().getValue();
    if (reverse) {
      location.transformBy(Pose2d.fromRotation(Rotation2d.fromDegrees(-180)));
    }

    Optional<Target> found = getNearestFacedTargetInternal(location);

    if (location.getTranslation().y() > 136) {
      Pose2d mirrored_location = location.mirror();
      Pose2d flipped_location =
          new Pose2d(
              mirrored_location.getTranslation().translateBy(new Translation2d(0, 324)),
              mirrored_location.getRotation());

      Optional<Target> other = getNearestFacedTargetInternal(flipped_location);

      if (found.isPresent() && other.isPresent()) {
        double d1 = location.distance(found.get().getTargetOrientation());
        double d2 = flipped_location.distance(other.get().getTargetOrientation());
        if (d2 < d1) {
          found = other;
        }
      } else if (found.isEmpty() && other.isPresent()) {
        found = other;
      }
    }

    return found.orElse(Target.NO_TARGET);
  }
}
