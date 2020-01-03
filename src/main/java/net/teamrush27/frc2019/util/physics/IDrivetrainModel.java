package net.teamrush27.frc2019.util.physics;

import java.text.DecimalFormat;
import net.teamrush27.frc2019.util.CSVWritable;

public interface IDrivetrainModel {

  ChassisState solveForwardKinematics(final WheelState wheel_motion);

  WheelState solveInverseKinematics(final ChassisState chassis_motion);

  DriveDynamics solveForwardDynamics(final ChassisState chassis_velocity, final WheelState voltage);

  DriveDynamics solveForwardDynamics(final WheelState wheel_velocity, final WheelState voltage);

  void solveForwardDynamics(DriveDynamics dynamics);

  DriveDynamics solveInverseDynamics(
      final ChassisState chassis_velocity, final ChassisState chassis_acceleration);

  DriveDynamics solveInverseDynamics(
      final WheelState wheel_velocity, final WheelState wheel_acceleration);

  void solveInverseDynamics(DriveDynamics dynamics);

  double getMaxAbsVelocity(double curvature, /*double dcurvature, */ double max_abs_voltage);

  MinMax getMinMaxAcceleration(
      final ChassisState chassis_velocity,
      double curvature, /*double dcurvature,*/
      double max_abs_voltage);

  class MinMax {

    public double min;
    public double max;
  }

  // Can refer to velocity or acceleration depending on context.
  class ChassisState implements CSVWritable {

    public double linear;
    public double angular;

    public ChassisState(double linear, double angular) {
      this.linear = linear;
      this.angular = angular;
    }

    public ChassisState() {}

    @Override
    public String toString() {
      DecimalFormat fmt = new DecimalFormat("#0.000");
      return fmt.format(linear) + ", " + fmt.format(angular);
    }

    @Override
    public String toCSV() {
      return this.toString();
    }

    @Override
    public String header(String base) {
      return null;
    }
  }

  // Can refer to velocity, acceleration, torque, voltage, etc., depending on context.
  class WheelState implements CSVWritable {

    public double left;
    public double right;

    public WheelState(double left, double right) {
      this.left = left;
      this.right = right;
    }

    public WheelState() {}

    public double get(boolean get_left) {
      return get_left ? left : right;
    }

    public void set(boolean set_left, double val) {
      if (set_left) {
        left = val;
      } else {
        right = val;
      }
    }

    @Override
    public String toString() {
      DecimalFormat fmt = new DecimalFormat("#0.000");
      return fmt.format(left) + ", " + fmt.format(right);
    }

    @Override
    public String toCSV() {
      return this.toString();
    }

    @Override
    public String header(String base) {
      return base + "_l," + base + "_r";
    }
  }

  // Full state dynamics of the drivetrain.
  // TODO maybe make these all optional fields and have a single solveDynamics() method that fills
  // in the blanks?
  class DriveDynamics implements CSVWritable {

    public double curvature = 0.0; // m^-1
    public double dcurvature = 0.0; // m^-1/m
    public IDrivetrainModel.ChassisState chassis_velocity =
        new IDrivetrainModel.ChassisState(); // m/s
    public IDrivetrainModel.ChassisState chassis_acceleration =
        new IDrivetrainModel.ChassisState(); // m/s^2
    public IDrivetrainModel.WheelState wheel_velocity = new IDrivetrainModel.WheelState(); // rad/s
    public IDrivetrainModel.WheelState wheel_acceleration =
        new IDrivetrainModel.WheelState(); // rad/s^2
    public IDrivetrainModel.WheelState voltage = new IDrivetrainModel.WheelState(); // V
    public IDrivetrainModel.WheelState wheel_torque = new IDrivetrainModel.WheelState(); // N m

    @Override
    public String toString() {
      DecimalFormat fmt = new DecimalFormat("#0.000");
      return fmt.format(curvature)
          + ", "
          + fmt.format(dcurvature)
          + ", ("
          + chassis_velocity
          + ", "
          + chassis_acceleration
          + "), ("
          + wheel_velocity
          + ", "
          + wheel_acceleration
          + ", "
          + voltage
          + ", "
          + wheel_torque
          + ")";
    }

    @Override
    public String toCSV() {
      return curvature
          + ","
          + dcurvature
          + ","
          + chassis_velocity.toCSV()
          + ", "
          + chassis_acceleration.toCSV()
          + ", "
          + wheel_velocity.toCSV()
          + ", "
          + wheel_acceleration.toCSV()
          + ", "
          + voltage.toCSV()
          + ", "
          + wheel_torque.toCSV();
    }

    @Override
    public String header(String base) {
      return base
          + "_curv,"
          + base
          + "_dcurv,"
          + chassis_velocity.header(base + "_chassis_v")
          + ","
          + chassis_acceleration.header(base + "_chassis_a")
          + ","
          + wheel_velocity.header(base + "_wheel_v")
          + ","
          + wheel_acceleration.header(base + "_wheel_a")
          + ","
          + voltage.header(base + "_voltage")
          + ","
          + wheel_torque.header(base + "_wheel_t");
    }
  }
}
