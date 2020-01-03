package net.teamrush27.frc2019.util.trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import net.teamrush27.frc2019.util.math.Pose2d;
import net.teamrush27.frc2019.util.math.Pose2dWithCurvature;
import net.teamrush27.frc2019.util.math.Rotation2d;
import net.teamrush27.frc2019.util.motion.DriveMotionPlanner;
import net.teamrush27.frc2019.util.trajectory.timing.CentripetalAccelerationConstraint;
import net.teamrush27.frc2019.util.trajectory.timing.TimedState;
import net.teamrush27.frc2019.util.trajectory.timing.TimingConstraint;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class TrajectoryGenerator {

  private static final Logger LOG = LogManager.getLogger(TrajectoryGenerator.class);

  private static final double kMaxVelocity = 120;
  private static final double kMaxAccel = 144;
  private static final double kMaxCentripetalAccelElevatorDown = 110.0;
  private static final double kMaxCentripetalAccel = 110.0;
  private static final double kMaxVoltage = 9.0;
  private static final double kFirstPathMaxVoltage = 9.0;
  private static final double kFirstPathMaxAccel = 130.0;
  private static final double kFirstPathMaxVel = 130.0;

  private static final double kSimpleSwitchMaxAccel = 100.0;
  private static final double kSimpleSwitchMaxCentripetalAccel = 80.0;
  private static final double kSimpleSwitchMaxVelocity = 120.0;

  private static TrajectoryGenerator INSTANCE = new TrajectoryGenerator();
  private final DriveMotionPlanner mMotionPlanner;
  private TrajectorySet mTrajectorySet = null;

  public static TrajectoryGenerator getInstance() {
    return INSTANCE;
  }

  private TrajectoryGenerator() {
    mMotionPlanner = new DriveMotionPlanner();
  }

  public void generateTrajectories() {
    if (mTrajectorySet == null) {
      LOG.info("Generating trajectories...");
      mTrajectorySet = new TrajectorySet();
      LOG.info("Finished trajectory generation");
    }
  }

  public TrajectorySet getTrajectorySet() {
    return mTrajectorySet;
  }

  public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
      boolean reversed,
      final List<Pose2d> waypoints,
      final List<TimingConstraint<Pose2dWithCurvature>> constraints,
      double max_vel, // inches/s
      double max_accel, // inches/s^2
      double max_voltage) {
    return mMotionPlanner.generateTrajectory(
        reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
  }

  public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
      boolean reversed,
      final List<Pose2d> waypoints,
      final List<TimingConstraint<Pose2dWithCurvature>> constraints,
      double start_vel, // inches/s
      double end_vel, // inches/s
      double max_vel, // inches/s
      double max_accel, // inches/s^2
      double max_voltage) {
    return mMotionPlanner.generateTrajectory(
        reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
  }

  private static final Pose2d trackingOffset = new Pose2d(-18, 0, Rotation2d.identity());
  private static final Pose2d bigTrackingOffset = new Pose2d(-36, 0, Rotation2d.identity());
  private static final Pose2d trackingReversedOffset = new Pose2d(24, 0, Rotation2d.identity());

  private static final Pose2d rotate180Offset = new Pose2d(0, 0, Rotation2d.fromDegrees(-180));

  private static final Pose2d smallRotationOffset = new Pose2d(0, 0, Rotation2d.fromDegrees(-11));

  // CRITICAL POSES
  // Origin is the center of the robot when the robot is placed against the middle of the alliance
  // station wall.
  // +x is towards the center of the field.
  // +y is to the right.
  // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON LEFT! (mirrored about +x axis for RIGHT)
  public static final Pose2d originPose = new Pose2d(0.0, 0.0, Rotation2d.identity());
  public static final Pose2d justStraightPose = new Pose2d(144.0, 0.0, Rotation2d.identity());
  public static final Pose2d justStraightBackPose = new Pose2d(-60.0, 0.0, Rotation2d.identity());
  public static final Pose2d turnRightBackPose =
      new Pose2d(-60.0, 60.0, Rotation2d.fromDegrees(-90));
  public static final Pose2d turnRightOrigin = new Pose2d(60.0, 0.0, Rotation2d.identity());
  public static final Pose2d turnRightCompletion =
      new Pose2d(120.0, -60.0, Rotation2d.fromDegrees(-90));
  public static final Pose2d turnLeftOrigin = new Pose2d(120.0, 120.0, Rotation2d.fromDegrees(90));
  public static final Pose2d turnLeftCompletion = new Pose2d(180.0, 180.0, Rotation2d.identity());
  public static final Pose2d turnRightLeftEnd = new Pose2d(240, 180, Rotation2d.identity());
  public static final Pose2d originToCloseSideCargoMid = new Pose2d(150, 0, Rotation2d.identity());
  public static final Pose2d originToCloseSideCargoPose =
      new Pose2d(240, 50, Rotation2d.fromDegrees(90));
  public static final Pose2d originToOffHab = new Pose2d(40, 5, Rotation2d.fromDegrees(10));
  public static final Pose2d originToFrontShipPose = new Pose2d(115, 30, Rotation2d.identity());

  public static final Pose2d originHabPose = new Pose2d(67, 114, Rotation2d.identity());

  public static final Pose2d offHABPose = new Pose2d(110, 114, Rotation2d.identity());
  public static final Pose2d rocketFarPrePose = new Pose2d(231, 68, Rotation2d.fromDegrees(-36));
  public static final Pose2d cargoSideClosePrePose =
      new Pose2d(150, 86, Rotation2d.fromDegrees(-27));
  public static final Pose2d cargoSideCloseReversePrePose =
      new Pose2d(205, 107, Rotation2d.fromDegrees(-6));

  public static final Pose2d humanPlayerStationPose = new Pose2d(17, 28, Rotation2d.identity());
  public static final Pose2d humanPlayerStationReversePose =
      new Pose2d(17, 35, Rotation2d.fromDegrees(-180));

  public static final Pose2d humanPlayerStationPoseOffset =
      new Pose2d(75, 40, Rotation2d.identity());
  public static final Pose2d humanPlayerStationPoseOffsetReverse =
      new Pose2d(100, 40, Rotation2d.fromDegrees(-180));

  public static final Pose2d humanPlayerCargoSideMidPose =
      new Pose2d(233, 64, Rotation2d.fromDegrees(11));
  public static final Pose2d humanPlayerCargoSideMidReversePose =
      new Pose2d(180, 50, Rotation2d.fromDegrees(-180));
  public static final Pose2d humanPlayerRocketSideMidPose =
      new Pose2d(206, 50, Rotation2d.fromDegrees(-30));
  public static final Pose2d humanPlayerRocketFarMidPose =
      new Pose2d(232, 55, Rotation2d.identity());

  public static final Pose2d cargoSideCloseHPMidPose =
      new Pose2d(200, 82, Rotation2d.fromDegrees(-162));

  public static final Pose2d cargoSideClosePose = new Pose2d(235, 85, Rotation2d.fromDegrees(90));
  public static final Pose2d cargoSideCloseReversePose =
      new Pose2d(250, 97, Rotation2d.fromDegrees(-90));

  public static final Pose2d cargoSideCloseFinishPose =
      new Pose2d(247, 95, Rotation2d.fromDegrees(95));

  public static final Pose2d cargoSideCloseOffsetReversePose =
      new Pose2d(251, 74, Rotation2d.fromDegrees(-90));
  public static final Pose2d cargoSideMidPose = new Pose2d(255, 97, Rotation2d.fromDegrees(90));
  public static final Pose2d cargoSideFarPose = new Pose2d(280, 97, Rotation2d.fromDegrees(90));

  public static final Pose2d cargoSideMidReversePose =
      new Pose2d(260, 97, Rotation2d.fromDegrees(-90));
  public static final Pose2d cargoSideMidReverseOffsetPose =
      new Pose2d(250, 87, Rotation2d.fromDegrees(-90));

  public static final Pose2d rocketFrontPose = new Pose2d(135, 57, Rotation2d.fromDegrees(-30));
  public static final Pose2d rocketSidePose = new Pose2d(230, 90, Rotation2d.fromDegrees(-90));
  public static final Pose2d rocketFarPose = new Pose2d(288, 45, Rotation2d.fromDegrees(29));

  public static final Pose2d rocketFarStartPose = new Pose2d(257, 27, Rotation2d.fromDegrees(29));
  public static final Pose2d midlinePose = new Pose2d(280, 70, Rotation2d.identity());

  public static final Pose2d rocketFarHPMidPose = new Pose2d(191, 55, Rotation2d.fromDegrees(-160));

  public class TrajectorySet {

    public class MirroredTrajectory {

      public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> left) {
        this.left = left;
        this.right = TrajectoryUtil.mirrorTimed(left);
      }

      public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
        return left ? this.left : this.right;
      }

      public Trajectory<TimedState<Pose2dWithCurvature>> getLeft() {
        return this.left;
      }

      public Trajectory<TimedState<Pose2dWithCurvature>> getRight() {
        return this.right;
      }

      public final Trajectory<TimedState<Pose2dWithCurvature>> left;
      public final Trajectory<TimedState<Pose2dWithCurvature>> right;
    }

    // For Trajectories that can run from either side, use MirroredTrajectory
    // For Trajectories that do not have a mirrored version (e.g. 1st stage switch autons),
    //    use Trajectory<TimedState<Pose2dWithCurvature>>

    public final MirroredTrajectory justStraight;
    public final MirroredTrajectory justStraightBack;
    public final MirroredTrajectory turnRightBack;
    public final MirroredTrajectory turnRight;
    public final MirroredTrajectory humanPlayerToCloseSideCargo;
    public final MirroredTrajectory habToFrontShip;

    public final MirroredTrajectory hpToRocketFront;
    public final MirroredTrajectory hpToCargoSideClose;
    public final MirroredTrajectory hpToCargoSideMid;
    public final MirroredTrajectory hpToCargoSideFar;

    public final MirroredTrajectory habToCargoSideClose;
    public final MirroredTrajectory habToRocketRear;

    public final MirroredTrajectory rocketRearToMidline;
    public final MirroredTrajectory midlineToHP;

    public final MirroredTrajectory cargoSideCloseToHP;

    public final MirroredTrajectory scootBack;
    public final MirroredTrajectory scootBack2;

    public final MirroredTrajectory altRocketRearToHP;

    public final MirroredTrajectory habToCargoFront;
    public final MirroredTrajectory cargoFrontToHP;

    private TrajectorySet() {
      // For Non-Mirrored Trajectories, just call the helper fn
      // For Mirrored Trajectories, wrap in `new MirroredTrajectory`

      LOG.info("justStraight");
      justStraight = new MirroredTrajectory(getJustStraight());
      LOG.info("justStraightBack");
      justStraightBack = new MirroredTrajectory(getJustStraightBack());
      LOG.info("turnRightBack");
      turnRightBack = new MirroredTrajectory(getTurnRightBack());

      LOG.info("turnRight");
      turnRight = new MirroredTrajectory(getTurnRight());
      LOG.info("humanPlayerToCloseSideCargo");
      humanPlayerToCloseSideCargo = new MirroredTrajectory(getHumanPlayerToCloseSideCargo());
      LOG.info("habToFrontShip");
      habToFrontShip = new MirroredTrajectory(getHabToFrontShip());

      LOG.info("hpToRocketHatchFront");
      hpToRocketFront = new MirroredTrajectory(getHPToRocketFront());
      LOG.info("hpToCargoSideClose");
      hpToCargoSideClose = new MirroredTrajectory(getHPToCargoSideClose());
      LOG.info("hpToCargoSideMid");
      hpToCargoSideMid = new MirroredTrajectory(getHPToCargoSideMid());
      LOG.info("hpToCargoSideFar");
      hpToCargoSideFar = new MirroredTrajectory(getHPToCargoSideFar());

      LOG.info("habToRocketRear");
      habToRocketRear = new MirroredTrajectory(getHABToRocketRear());
      LOG.info("habToCargoSideClose");
      habToCargoSideClose = new MirroredTrajectory(getHABToCargoSideClose());

      LOG.info("rocketRearToMidline");
      rocketRearToMidline = new MirroredTrajectory(getRocketRearToMidline());
      LOG.info("midlineToHP");
      midlineToHP = new MirroredTrajectory(getMidlineToHP());

      LOG.info("cargoSideCloseToHP");
      cargoSideCloseToHP = new MirroredTrajectory(getCargoSideCloseToHP());

      LOG.info("scootBack");
      scootBack = new MirroredTrajectory(getScootBack());

      LOG.info("scootBack2");
      scootBack2 = new MirroredTrajectory(getScootBack2());

      LOG.info("altRocketRearToHP");
      altRocketRearToHP = new MirroredTrajectory(getAltRocketRearToHP());

      LOG.info("habToCargoFront");
      habToCargoFront = new MirroredTrajectory(getHabToCargoFront());

      LOG.info("cargoFrontToHP");
      cargoFrontToHP = new MirroredTrajectory(getCargoFrontToHP());
    }

    public void saveTrajectories() {
      hpToRocketFront.getRight().save("HP_TO_ROCKET_FRONT");
      hpToCargoSideClose.getRight().save("HP_TO_CARGO_SIDE_CLOSE");
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getJustStraight() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(originPose);
      waypoints.add(justStraightPose);

      return generateTrajectory(
          false,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          kMaxVelocity,
          kMaxAccel,
          kMaxVoltage);
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getJustStraightBack() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(originPose);
      waypoints.add(justStraightBackPose);

      return generateTrajectory(
          true,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          kMaxVelocity,
          kMaxAccel,
          kMaxVoltage);
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getTurnRightBack() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(originPose);
      waypoints.add(turnRightBackPose);

      return generateTrajectory(
          true,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          kMaxVelocity,
          kMaxAccel,
          kMaxVoltage);
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getTurnRight() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(originPose);
      waypoints.add(turnRightOrigin);
      waypoints.add(turnRightCompletion);
      // waypoints.add(turnLeftOrigin);
      // waypoints.add(turnLeftCompletion);
      // waypoints.add(turnRightLeftEnd);

      return generateTrajectory(
          false,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          kMaxVelocity,
          kMaxAccel,
          kMaxVoltage);
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getHumanPlayerToCloseSideCargo() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(originPose);
      waypoints.add(originToCloseSideCargoMid);
      waypoints.add(originToCloseSideCargoPose);

      return generateTrajectory(
          false,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          kMaxVelocity,
          kMaxAccel,
          kMaxVoltage);
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getHabToFrontShip() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(originPose);
      waypoints.add(originToOffHab);
      waypoints.add(originToFrontShipPose);

      return generateTrajectory(
          false,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          kMaxVelocity,
          kMaxAccel,
          kMaxVoltage);
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getFrontShipToHumanPlayer() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(originPose);
      waypoints.add(originToOffHab);
      waypoints.add(originToFrontShipPose);

      return generateTrajectory(
          false,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          kMaxVelocity,
          kMaxAccel,
          kMaxVoltage);
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getHPToRocketFront() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(humanPlayerStationReversePose);
      waypoints.add(
          rocketFrontPose.transformBy(
              rotate180Offset)); // .transformBy(trackingOffset).transformBy(rotate180Offset));
      // waypoints.add(rocketFrontPose);

      return generateTrajectory(
          true,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          kMaxVelocity,
          kMaxAccel,
          kMaxVoltage);
    }

    private final Pose2d cargoShipCloseFirstMidpoint =
        new Pose2d(66, 50, Rotation2d.fromDegrees(22));
    private final Pose2d cargoShipCloseSecondMidpoint = new Pose2d(165, 60, Rotation2d.identity());

    private Trajectory<TimedState<Pose2dWithCurvature>> getHPToCargoSideClose() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(humanPlayerStationPose);
      waypoints.add(cargoShipCloseFirstMidpoint);
      waypoints.add(cargoShipCloseSecondMidpoint);
      waypoints.add(cargoSideClosePose);

      return generateTrajectory(
          false,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          kMaxVelocity,
          kMaxAccel,
          kMaxVoltage);
    }

    private final Pose2d fixitPose = new Pose2d(148, 54, Rotation2d.fromDegrees(9));

    private Trajectory<TimedState<Pose2dWithCurvature>> getHPToCargoSideMid() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(humanPlayerStationPose);
      waypoints.add(fixitPose);
      waypoints.add(humanPlayerCargoSideMidPose);
      waypoints.add(cargoSideMidPose);

      return generateTrajectory(
          false,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          kMaxVelocity,
          kMaxAccel,
          kMaxVoltage);
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getHPToCargoSideFar() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(humanPlayerStationPose);
      waypoints.add(humanPlayerCargoSideMidPose);
      waypoints.add(cargoSideFarPose.transformBy(trackingOffset));

      return generateTrajectory(
          false,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          kMaxVelocity,
          kMaxAccel,
          kMaxVoltage);
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getHABToRocketRear() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(originHabPose);
      // waypoints.add(offHABPose);
      waypoints.add(rocketFarPrePose);
      waypoints.add(rocketFarPose);

      return generateTrajectory(
          false,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          kMaxVelocity,
          kMaxAccel,
          kMaxVoltage);
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getHABToCargoSideClose() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(originHabPose);
      // waypoints.add(offHABPose.transformBy(smallRotationOffset));
      waypoints.add(cargoSideClosePrePose);
      waypoints.add(cargoSideCloseFinishPose);

      return generateTrajectory(
          false,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(80)),
          kMaxVelocity,
          kMaxAccel,
          kMaxVoltage);
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getRocketRearToMidline() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(rocketFarStartPose);
      waypoints.add(midlinePose);

      return generateTrajectory(
          false,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          kMaxVelocity,
          kMaxAccel,
          kMaxVoltage);
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getMidlineToHP() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(midlinePose);
      waypoints.add(humanPlayerStationPoseOffset);

      return generateTrajectory(
          true,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          kMaxVelocity,
          200,
          kMaxVoltage);
    }

    private final Pose2d dumbOffset = new Pose2d(48, 15, Rotation2d.identity());

    private Trajectory<TimedState<Pose2dWithCurvature>> getCargoSideCloseToHP() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(cargoSideCloseReversePose.transformBy(rotate180Offset));
      waypoints.add(cargoSideCloseHPMidPose.transformBy(rotate180Offset));
      waypoints.add(humanPlayerStationPoseOffset.transformBy(dumbOffset));

      return generateTrajectory(
          true,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          kMaxVelocity,
          kMaxAccel,
          kMaxVoltage);
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getScootBack() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(rocketFrontPose.transformBy(rotate180Offset));
      waypoints.add(
          rocketFrontPose.transformBy(rotate180Offset).transformBy(trackingReversedOffset));

      return generateTrajectory(
          false,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          36,
          36,
          kMaxVoltage);
    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getScootBack2() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(cargoSideMidReversePose);
      waypoints.add(cargoSideMidReversePose.transformBy(trackingReversedOffset));

      return generateTrajectory(
          false,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          kMaxVelocity,
          kMaxAccel,
          kMaxVoltage);
    }

    private final Pose2d someDumbPose = new Pose2d(257, 71, Rotation2d.fromDegrees(-180));
    private final Pose2d otherDumbPose = new Pose2d(6, 0, Rotation2d.identity());

    private Trajectory<TimedState<Pose2dWithCurvature>> getAltRocketRearToHP() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(rocketFarStartPose);
      waypoints.add(rocketFarStartPose.transformBy(otherDumbPose));
      waypoints.add(someDumbPose);
      // waypoints.add(rocketFarHPMidPose);
      waypoints.add(humanPlayerStationPoseOffsetReverse);

      return generateTrajectory(
          false,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          kMaxVelocity,
          kMaxAccel,
          kMaxVoltage);
    }

    private final Pose2d originCenterHabPose = new Pose2d(67, 145, Rotation2d.identity());
    private final Pose2d cargoFrontEndPose = new Pose2d(150, 145, Rotation2d.identity());

    private Trajectory<TimedState<Pose2dWithCurvature>> getHabToCargoFront() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(originCenterHabPose);
      // waypoints.add(offHABPose);
      waypoints.add(cargoFrontEndPose);

      return generateTrajectory(
          false,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          kMaxVelocity,
          kMaxAccel,
          kMaxVoltage);
    }

    private final Pose2d cargoFrontStartPose = new Pose2d(190, 145, Rotation2d.identity());
    private final Pose2d cargoFrontHPMidPose = new Pose2d(140, 90, Rotation2d.fromDegrees(70));
    private final Pose2d thisIsTheWorstPoint = new Pose2d(100, 40, Rotation2d.identity());

    private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFrontToHP() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(cargoFrontStartPose);
      waypoints.add(cargoFrontHPMidPose);
      waypoints.add(thisIsTheWorstPoint);

      return generateTrajectory(
          true,
          waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
          kMaxVelocity,
          kMaxAccel,
          kMaxVoltage);
    }
  }
}
