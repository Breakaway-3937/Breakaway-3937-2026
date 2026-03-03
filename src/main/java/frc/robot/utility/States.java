package frc.robot.utility;

public class States {

    public enum IntakeStates {
        STOW(400.0, 0.0),
        FIRE(212.0,60.0),
        IDLE(212.0, 0.0),
        INTAKE(20.0, 18.0),
        UNCLOG(20.0, -10.0);

        private final double angle, power;
        private final double INTAKE_POWER_CONVERSION = 1.67;
        private final double INTAKE_WRIST_CONVERSION = 0.025;

        private IntakeStates(double angle, double power) {
            this.angle = angle;
            this.power = power;
        }

        public double getAngle() {
            return angle * INTAKE_WRIST_CONVERSION;
        }

        public double getPower() {
            return power * INTAKE_POWER_CONVERSION;
        }
    }

    public enum IndexerStates {
        IDLE(0.0, 0.0),
        FIRE(3.0, 400.0);

        private final double SpinnerSpeed, KickerSpeed;
        private final double SPINNER_CONVERSION = 12.67;

        private IndexerStates(double SpinnerSpeed, double KickerSpeed) {
            this.SpinnerSpeed = SpinnerSpeed;
            this.KickerSpeed = KickerSpeed;
        }

        public double getSpinnerSpeed() {
            return SpinnerSpeed * SPINNER_CONVERSION;
        }

        public double getKickerSpeed() {
            return KickerSpeed;
        }
    }

    public enum ClimberStates {
        STOW(0.0),
        RUNG_ONE(0.0),
        RUNG_TWO(0.0),
        RUNG_THREE(0.0),
        PRESTAGE(0.0),
        PULL(0.0);

        private final double Climb;

        private ClimberStates(double Climb) {
            this.Climb = Climb;
        }

        public double getClimb() {
            return Climb;
        }

    }
}