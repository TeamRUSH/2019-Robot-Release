package net.teamrush27.frc2019.base;

public class RobotMap {

  // Drive
  public static final int DRIVE_RIGHT_MASTER_CAN_ID = 11;
  public static final int DRIVE_RIGHT_SLAVE_1_CAN_ID = 15;
  //public static final int DRIVE_RIGHT_SLAVE_2_CAN_ID = 15;
  public static final int DRIVE_LEFT_MASTER_CAN_ID = 12;
  public static final int DRIVE_LEFT_SLAVE_1_CAN_ID = 16;
  //public static final int DRIVE_LEFT_SLAVE_2_CAN_ID = 16;
  public static final int ARM_ROTATION_SENSOR_CAN_ID = 14;

  public static final int LEFT_DRIVE_SHIFT_SERVO_ID = 0;
  public static final int RIGHT_DRIVE_SHIFT_SERVO_ID = 1;

  // Arm
  public static final int ARM_ROTATION_MASTER_CAN_ID = 17;
  public static final int ARM_ROTATION_SLAVE_CAN_ID = 18;
  public static final int ARM_ROTATION_HOME_SENSOR_ID = 4;

  public static final int ARM_EXTENSION_CAN_ID = 19;
  public static final int ARM_EXTENSION_HOME_SENSOR_ID = 3;
  public static final int ARM_EXTENSION_FULL_EXT_SENSOR_ID = 2;

  // Gripper
  public static final int GRIPPER_JAWS_CAN_ID = 20;
  public static final int GRIPPER_MOTOR_CAN_ID = 22;
  public static final int GRIPPER_MOTOR_SPARK_CAN_ID = 28;

  public static final int GRIPPER_JAWS_TOP_SERVO_ID = 2;
  public static final int GRIPPER_JAWS_BOTTOM_SERVO_ID = 3;

  public static final int GRIPPER_CARGO_ANALOG_SENSOR_ID = 1;
  public static final int GRIPPER_HATCH_DIGITAL_SENSOR_ID = 0;
  public static final int GRIPPER_JAW_HOME_SENSOR_ID = 1;
  public static final int GRIPPER_JAW_MAX_SENSOR_ID = 2;

  // Wrist
  public static final int WRIST_MOTOR_CAN_ID = 23;
  public static final int WRIST_CANIFIER_CAN_ID = 30;

  // Spider Legs
  public static final int REAR_SPIDER_LEG_MOTOR_MASTER_CAN_ID = 24;
  public static final int REAR_SPIDER_LEG_MOTOR_SLAVE_CAN_ID = 21;
  public static final int REAR_SPIDER_LEG_HOME_SENSOR_ID = 6;

  public static final int FRONT_SPIDER_LEG_MOTOR_CAN_ID = 25;
  public static final int FRONT_SPIDER_LEG_HOME_SENSOR_ID = 5;

  public static final int SPIDER_LEG_FRONT_FLOOR_SENSOR_ID = 2;
  public static final int SPIDER_LEG_REAR_FLOOR_SENSOR_ID = 3;
}
