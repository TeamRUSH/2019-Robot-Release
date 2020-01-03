package net.teamrush27.frc2019.auto.modes;

import java.util.ArrayList;
import java.util.List;
import net.teamrush27.frc2019.auto.AutoModeBase;
import net.teamrush27.frc2019.auto.AutoModeEndedException;
import net.teamrush27.frc2019.auto.actions.impl.CollectAccelerationData;
import net.teamrush27.frc2019.auto.actions.impl.CollectVelocityData;
import net.teamrush27.frc2019.auto.actions.impl.WaitAction;
import net.teamrush27.frc2019.util.physics.DriveCharacterization;
import net.teamrush27.frc2019.util.physics.DriveCharacterization.VelocityDataPoint;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class CharacterizeDrivetrain extends AutoModeBase {
  private static final Logger LOG = LogManager.getLogger(CharacterizeDrivetrain.class);

  @Override
  protected void routine() throws AutoModeEndedException {
    List<VelocityDataPoint> velocityData = new ArrayList<>();
    List<DriveCharacterization.AccelerationDataPoint> accelerationData = new ArrayList<>();

    // runAction(new ShiftHighGearAction(false));
    // runAction(new WaitAction(10));

    runAction(new CollectVelocityData(velocityData, false, false));
    runAction(new WaitAction(20));
    runAction(new CollectAccelerationData(accelerationData, false, false));

    DriveCharacterization.CharacterizationConstants constants =
        DriveCharacterization.characterizeDrive(velocityData, accelerationData);

    LOG.info("ks: {}", constants.ks);
    LOG.info("kv: {}", constants.kv);
    LOG.info("ka: {}", constants.ka);
  }
}
