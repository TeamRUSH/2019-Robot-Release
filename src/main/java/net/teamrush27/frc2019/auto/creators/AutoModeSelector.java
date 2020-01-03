package net.teamrush27.frc2019.auto.creators;

import com.google.gson.Gson;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import net.teamrush27.frc2019.auto.AutoModeBase;
import net.teamrush27.frc2019.auto.modes.CharacterizeDrivetrain;
import net.teamrush27.frc2019.auto.modes.DoNothing;
import net.teamrush27.frc2019.auto.modes.RightCargo;
import net.teamrush27.frc2019.auto.modes.RightCloseCargo;
import net.teamrush27.frc2019.auto.modes.RightRocket;
import net.teamrush27.frc2019.auto.modes.TurnInPlace;
import net.teamrush27.frc2019.auto.modes.singlepath.AltRocketReturnPathRight;
import net.teamrush27.frc2019.auto.modes.singlepath.CargoFrontToHPRight;
import net.teamrush27.frc2019.auto.modes.singlepath.CargoSideCloseToHPRight;
import net.teamrush27.frc2019.auto.modes.singlepath.HPToCargoSideCloseRight;
import net.teamrush27.frc2019.auto.modes.singlepath.HPToCargoSideMidRight;
import net.teamrush27.frc2019.auto.modes.singlepath.HPToRocketCloseRight;
import net.teamrush27.frc2019.auto.modes.singlepath.HabToCargoFrontRight;
import net.teamrush27.frc2019.auto.modes.singlepath.HabToCargoSideCloseRight;
import net.teamrush27.frc2019.auto.modes.singlepath.HabToRocketRearRight;

public class AutoModeSelector {

  public static final String AUTO_OPTIONS_DASHBOARD_KEY = "auto_modes";
  public static final String SELECTED_AUTO_MODE_DASHBOARD_KEY = "selected_auto";
  public static final String ROBOT_SELECTED_AUTO_MODE_DASHBOARD_KEY = "robot_selected_auto";

  public static void update() {
    String selectedModeName =
        SmartDashboard.getString(SELECTED_AUTO_MODE_DASHBOARD_KEY, "NO SELECTED MODE!!!!");
    AutoModeCreator autoModeCreator = null;

    for (AutoModeCreator mode : ALL_MODES) {
      if (mode.getDashboardName().equals(selectedModeName)) {
        autoModeCreator = mode;
      }
    }

    if (autoModeCreator == null) {
      autoModeCreator = DEFAULT_MODE;
    }

    SmartDashboard.putString(ROBOT_SELECTED_AUTO_MODE_DASHBOARD_KEY, autoModeCreator.dashboardName);
  }

  private static class AutoModeCreator {

    private final String dashboardName;
    private final Supplier<AutoModeBase> creator;

    private AutoModeCreator(String dashboardName, Supplier<AutoModeBase> creator) {
      this.dashboardName = dashboardName;
      this.creator = creator;
    }

    public Supplier<AutoModeBase> getCreator() {
      return creator;
    }

    public String getDashboardName() {
      return dashboardName;
    }
  }

  private static final AutoModeCreator DEFAULT_MODE =
      new AutoModeCreator("Nothing", () -> new DoNothing());

  private static final AutoModeCreator[] ALL_MODES = {
    new AutoModeCreator("Cargo Ship", () -> new RightCargo()),
    new AutoModeCreator("Rocket", () -> new RightRocket()),
    new AutoModeCreator("Close Cargo Ship", () -> new RightCloseCargo()),
    new AutoModeCreator("Nothing", () -> new DoNothing()),
    new AutoModeCreator("Characterize", () -> new CharacterizeDrivetrain()),
    new AutoModeCreator("Turn In Place", () -> new TurnInPlace()),
    new AutoModeCreator("Hab -> Cargo Side Close Right", () -> new HabToCargoSideCloseRight()),
    new AutoModeCreator("Cargo Side Close -> HP Right", () -> new CargoSideCloseToHPRight()),
    new AutoModeCreator("HP -> Cargo Side Mid Right", () -> new HPToCargoSideMidRight()),
    new AutoModeCreator("Hab -> Rocket Rear Right", () -> new HabToRocketRearRight()),
    new AutoModeCreator("HP -> Rocket Side Close Right", () -> new HPToRocketCloseRight()),
    new AutoModeCreator("Rocket Rear -> HP Right", () -> new AltRocketReturnPathRight()),
    new AutoModeCreator("Hab -> Cargo Front Right", () -> new HabToCargoFrontRight()),
    new AutoModeCreator("Cargo Front -> HP Right", () -> new CargoFrontToHPRight()),
    new AutoModeCreator("HP -> Cargo Side Close Right", () -> new HPToCargoSideCloseRight()),
    // new AutoModeCreator("LEFT Cargo Ship", () -> new LeftCargo()),
    // new AutoModeCreator("LEFT Rocket", () -> new LeftRocket()),
    // new AutoModeCreator("LEFT Close Cargo Ship", () -> new LeftCloseCargo()),

  };

  public static void initAutoModeSelector() {

    List<String> modeNames = new ArrayList<String>();

    for (AutoModeCreator mode : ALL_MODES) {
      modeNames.add(mode.getDashboardName());
    }

    final Gson gson = new Gson();

    String modesJSON = gson.toJson(modeNames);

    SmartDashboard.putString(AUTO_OPTIONS_DASHBOARD_KEY, modesJSON);
    SmartDashboard.putString(SELECTED_AUTO_MODE_DASHBOARD_KEY, DEFAULT_MODE.getDashboardName());
  }

  public static AutoModeBase getSelectedAutoMode() {
    String selectedModeName =
        SmartDashboard.getString(SELECTED_AUTO_MODE_DASHBOARD_KEY, "NO SELECTED MODE!!!!");
    for (AutoModeCreator mode : ALL_MODES) {
      if (mode.getDashboardName().equals(selectedModeName)) {
        return mode.getCreator().get();
      }
    }
    DriverStation.reportError("Failed to select auto mode: " + selectedModeName, false);
    return DEFAULT_MODE.getCreator().get();
  }
}
