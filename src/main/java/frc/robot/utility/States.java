package frc.robot.utility;

public class States {

    public enum IntakeStates {
        STOW(10.0, 0.0),
        FIRE(4.0,35.0),
        IDLE(4.0, 0.0),
        INTAKE(0.5, 35.0),
        UNCLOG(0.5, -10.0);

        private final double angle, power;

        private IntakeStates(double angle, double power) {
            this.angle = angle;
            this.power = power;
        }

        public double getAngle() {
            return angle;
        }

        public double getPower() {
            return power;
        }
    }

    public enum IndexerStates {
        IDLE(0.0, 0.0),
        FIRE(10.0, 1000.0);

        private final double SpinnerSpeed, KickerSpeed;

        private IndexerStates(double SpinnerSpeed, double KickerSpeed) {
            this.SpinnerSpeed = SpinnerSpeed;
            this.KickerSpeed = KickerSpeed;
        }

        public double getSpinnerSpeed() {
            return SpinnerSpeed;
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