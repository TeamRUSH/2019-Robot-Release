package net.teamrush27.frc2019.wrappers;

import edu.wpi.first.wpilibj.GenericHID;

public class XboxController extends GenericHID {

    public  XboxController(int port) {
        super(port);
    }

    private enum Button {
        kA(1),
        kB(2),
        kX(3),
        kY(4),
        kLB(5),
        kRB(6),
        kSelect(7),
        kStart(8),
        kLeftStick(9),
        kRightStick(10);

        @SuppressWarnings({"MemberName", "PMD.SingularField"})
		private final int value;
		
		Button(int value) {
			this.value = value;
		}
    }

    @Override
    public double getX(Hand hand) {
        if (hand.equals(Hand.kLeft)) {
            return getRawAxis(0);
        } else {
            return getRawAxis(4);
        }
    }

    @Override
    public double getY(Hand hand) {
        if (hand.equals(Hand.kLeft)) {
            return getRawAxis(1);
        } else {
            return getRawAxis(5);
        }
    }

    public double getTriggerAxis(Hand hand) {
		if (hand.equals(Hand.kLeft)) {
			return getRawAxis(2);
		} else {
			return getRawAxis(3);
		}
	}
    
    public boolean getBumper(Hand hand) {
        if (hand.equals(Hand.kLeft)) {
            return getRawButton(Button.kLB.value);
        } else {
            return getRawButton(Button.kRB.value);
        }
    }

    public boolean getBumperPressed(Hand hand) {
        if (hand == Hand.kLeft) {
            return getRawButtonPressed(Button.kLB.value);
        } else {
            return getRawButtonPressed(Button.kRB.value);
        }
    }

    public boolean getBumperReleased(Hand hand) {
        if (hand == Hand.kLeft) {
            return getRawButtonReleased(Button.kLB.value);
        } else {
            return getRawButtonReleased(Button.kRB.value);
        }
    }

    public boolean getStickButton(Hand hand) {
        if (hand.equals(Hand.kLeft)) {
            return getRawButton(Button.kLeftStick.value);
        } else {
            return getRawButton(Button.kRightStick.value);
        }
    }

    public boolean getStickButtonPressed(Hand hand) {
        if (hand == Hand.kLeft) {
            return getRawButtonPressed(Button.kLeftStick.value);
        } else {
            return getRawButtonPressed(Button.kRightStick.value);
        }
    }

    public boolean getStickButtonReleased(Hand hand) {
        if (hand == Hand.kLeft) {
            return getRawButtonReleased(Button.kLeftStick.value);
        } else {
            return getRawButtonReleased(Button.kRightStick.value);
        }
    }

    public boolean getAButton() {
        return getRawButton(Button.kA.value);
    }

    public boolean getAButtonPressed() {
        return getRawButtonPressed(Button.kA.value);
    }

    public boolean getAButtonReleased() {
        return getRawButtonReleased(Button.kA.value);
    }

    public boolean getBButton() {
        return getRawButton(Button.kB.value);
    }

    public boolean getBButtonPressed() {
        return getRawButtonPressed(Button.kB.value);
    }

    public boolean getBButtonReleased() {
        return getRawButtonReleased(Button.kB.value);
    }

    public boolean getXButton() {
        return getRawButton(Button.kX.value);
    }

    public boolean getXButtonPressed() {
        return getRawButtonPressed(Button.kX.value);
    }

    public boolean getXButtonReleased() {
        return getRawButtonReleased(Button.kX.value);
    }

    public boolean getYButton() {
        return getRawButton(Button.kY.value);
    }

    public boolean getYButtonPressed() {
        return getRawButtonPressed(Button.kY.value);
    }

    public boolean getYButtonReleased() {
        return getRawButtonReleased(Button.kY.value);
    }

    public boolean getSelectButton() {
        return getRawButton(Button.kSelect.value);
    }

    public boolean getSelectButtonPressed() {
        return getRawButtonPressed(Button.kSelect.value);
    }

    public boolean getSelectButtonReleased() {
        return getRawButtonReleased(Button.kSelect.value);
    }

    public boolean getStartButton() {
        return getRawButton(Button.kStart.value);
    }

    public boolean getStartButtonPressed() {
        return getRawButtonPressed(Button.kStart.value);
    }

    public boolean getStartButtonReleased() {
        return getRawButtonReleased(Button.kStart.value);
    }
}