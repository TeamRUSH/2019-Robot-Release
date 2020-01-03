package net.teamrush27.frc2019.util.motion;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import net.teamrush27.frc2019.util.math.MathUtils;

/**
 * A motion profile specifies a 1D time-parameterized trajectory. The trajectory is composed of
 * successively coincident MotionSegments from which the desired state of motion at any given
 * distance or time can be calculated.
 */
public class MotionProfile {
  protected List<MotionSegment> segments;

  /** Create an empty MotionProfile. */
  public MotionProfile() {
    segments = new ArrayList<>();
  }

  /**
   * Create a MotionProfile from an existing list of segments (note that validity is not checked).
   *
   * @param segments The new segments of the profile.
   */
  public MotionProfile(List<MotionSegment> segments) {
    this.segments = segments;
  }

  /**
   * Checks if the given MotionProfile is valid. This checks that:
   *
   * <p>1. All segments are valid.
   *
   * <p>2. Successive segments are C1 continuous in position and C0 continuous in velocity.
   *
   * @return True if the MotionProfile is valid.
   */
  public boolean isValid() {
    MotionSegment previousSegment = null;
    for (MotionSegment segment : segments) {
      if (!segment.isValid()) {
        return false;
      }
      if (previousSegment != null && !segment.start().coincident(previousSegment.end())) {
        // Adjacent segments are not continuous.
        System.err.println(
            "Segments not continuous! End: "
                + previousSegment.end()
                + ", Start: "
                + segment.start());
        return false;
      }
      previousSegment = segment;
    }
    return true;
  }

  /**
   * Check if the profile is empty.
   *
   * @return True if there are no segments.
   */
  public boolean isEmpty() {
    return segments.isEmpty();
  }

  /**
   * Get the interpolated MotionState at any given time.
   *
   * @param time The time to query.
   * @return Empty if the time is outside the time bounds of the profile, or the resulting
   *     MotionState otherwise.
   */
  public Optional<MotionState> stateByTime(double time) {
    if (time < startTime() && time + MathUtils.DEFAULT_MAX_ERROR >= startTime()) {
      return Optional.of(startState());
    }
    if (time > endTime() && time - MathUtils.DEFAULT_MAX_ERROR <= endTime()) {
      return Optional.of(endState());
    }
    for (MotionSegment segment : segments) {
      if (segment.containsTime(time)) {
        return Optional.of(segment.start().extrapolate(time));
      }
    }
    return Optional.empty();
  }

  /**
   * Get the interpolated MotionState at any given time, clamping to the endpoints if time is out of
   * bounds.
   *
   * @param time The time to query.
   * @return The MotionState at time t, or closest to it if t is outside the profile.
   */
  public MotionState stateByTimeClamped(double time) {
    if (time < startTime()) {
      return startState();
    } else if (time > endTime()) {
      return endState();
    }
    for (MotionSegment segment : segments) {
      if (segment.containsTime(time)) {
        return segment.start().extrapolate(time);
      }
    }
    // Should never get here.
    return MotionState.INVALID_STATE;
  }

  /**
   * Get the interpolated MotionState by distance (the "pos()" field of MotionState). Note that
   * since a profile may reverse, this method only returns the *first* instance of this position.
   *
   * @param position The position to query.
   * @return Empty if the profile never crosses pos or if the profile is invalid, or the resulting
   *     MotionState otherwise.
   */
  public Optional<MotionState> firstStateByPos(double position) {
    for (MotionSegment segment : segments) {
      if (segment.containsPosition(position)) {
        if (MathUtils.epsilonEquals(
            segment.end().position(), position, MathUtils.DEFAULT_MAX_ERROR)) {
          return Optional.of(segment.end());
        }
        final double time = Math.min(segment.start().nextTimeAtPos(position), segment.end().time());
        if (Double.isNaN(time)) {
          System.err.println("Error! We should reach 'pos' but we don't");
          return Optional.empty();
        }
        return Optional.of(segment.start().extrapolate(time));
      }
    }
    // We never reach pos.
    return Optional.empty();
  }

  /**
   * Remove all parts of the profile prior to the query time. This eliminates whole segments and
   * also shortens any segments containing t.
   *
   * @param time The query time.
   */
  public void trimBeforeTime(double time) {
    for (Iterator<MotionSegment> iterator = segments.iterator(); iterator.hasNext(); ) {
      MotionSegment segment = iterator.next();
      if (segment.end().time() <= time) {
        // Segment is fully before t.
        iterator.remove();
        continue;
      }
      if (segment.start().time() <= time) {
        // Segment begins before t; let's shorten the segment.
        segment.setStart(segment.start().extrapolate(time));
      }
      break;
    }
  }

  /** Remove all segments. */
  public void clear() {
    segments.clear();
  }

  /**
   * Remove all segments and initialize to the desired state (actually a segment of length 0 that
   * starts and ends at initial_state).
   *
   * @param initalState The MotionState to initialize to.
   */
  public void reset(MotionState initalState) {
    clear();
    segments.add(new MotionSegment(initalState, initalState));
  }

  /** Remove redundant segments (segments whose start and end states are coincident). */
  public void consolidate() {
    for (Iterator<MotionSegment> iterator = segments.iterator();
        iterator.hasNext() && segments.size() > 1; ) {
      MotionSegment segment = iterator.next();
      if (segment.start().coincident(segment.end())) {
        iterator.remove();
      }
    }
  }

  /**
   * Add to the profile by applying an acceleration control for a given time. This is appended to
   * the previous last state.
   *
   * @param acceleration The acceleration to apply.
   * @param deltaTime The period of time to apply the given acceleration.
   */
  public void appendControl(double acceleration, double deltaTime) {
    if (isEmpty()) {
      System.err.println("Error!  Trying to append to empty profile");
      return;
    }
    MotionState lastEndState = segments.get(segments.size() - 1).end();
    MotionState newStartState =
        new MotionState(
            lastEndState.time(), lastEndState.position(), lastEndState.velocity(), acceleration);
    appendSegment(
        new MotionSegment(
            newStartState, newStartState.extrapolate(newStartState.time() + deltaTime)));
  }

  /**
   * Add to the profile by inserting a new segment. No validity checking is done.
   *
   * @param segment The segment to add.
   */
  public void appendSegment(MotionSegment segment) {
    segments.add(segment);
  }

  /**
   * Add to the profile by inserting a new profile after the final state. No validity checking is
   * done.
   *
   * @param profile The profile to add.
   */
  public void appendProfile(MotionProfile profile) {
    for (MotionSegment segment : profile.segments()) {
      appendSegment(segment);
    }
  }

  /** @return The number of segments. */
  public int size() {
    return segments.size();
  }

  /** @return The list of segments. */
  public List<MotionSegment> segments() {
    return segments;
  }

  /** @return The first state in the profile (or kInvalidState if empty). */
  public MotionState startState() {
    if (isEmpty()) {
      return MotionState.INVALID_STATE;
    }
    return segments.get(0).start();
  }

  /** @return The time of the first state in the profile (or NaN if empty). */
  public double startTime() {
    return startState().time();
  }

  /** @return The pos of the first state in the profile (or NaN if empty). */
  public double startPos() {
    return startState().position();
  }

  /** @return The last state in the profile (or kInvalidState if empty). */
  public MotionState endState() {
    if (isEmpty()) {
      return MotionState.INVALID_STATE;
    }
    return segments.get(segments.size() - 1).end();
  }

  /** @return The time of the last state in the profile (or NaN if empty). */
  public double endTime() {
    return endState().time();
  }

  /** @return The pos of the last state in the profile (or NaN if empty). */
  public double endPos() {
    return endState().position();
  }

  /** @return The duration of the entire profile. */
  public double duration() {
    return endTime() - startTime();
  }

  /**
   * @return The total distance covered by the profile. Note that distance is the sum of absolute
   *     distances of all segments, so a reversing profile will count the distance covered in each
   *     direction.
   */
  public double length() {
    double length = 0.0;
    for (MotionSegment segment : segments()) {
      length += Math.abs(segment.end().position() - segment.start().position());
    }
    return length;
  }

  @Override
  public String toString() {
    StringBuilder builder = new StringBuilder("Profile:");
    for (MotionSegment segment : segments()) {
      builder.append("\n\t");
      builder.append(segment);
    }
    return builder.toString();
  }
}
