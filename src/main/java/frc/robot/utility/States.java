package frc.robot.utility;

public class States {

    public enum IntakeStates {
        STOW(400.0, 0.0),
        FIRE(212.0,21.0),
        IDLE(212.0, 0.0),
        INTAKE(0.5, 35.0),
        UNCLOG(0.5, -40.0);


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
        IDLE(0.0, 0.0, 0.0, 0.0),
        FIRE(7.0, 40.0, 50.0, 50.0),
        SHUNCLOG(7.0, 40.0, 50.0, 50.0);

        private final double SpinnerSpeed, KickerSpeed, UpsySpeed, DiverterSpeed;
        private final double SPINNER_CONVERSION = 12.67;

        private IndexerStates(double SpinnerSpeed, double KickerSpeed, double UpsySpeed, double DiverterSpeed) {
            this.SpinnerSpeed = SpinnerSpeed;
            this.KickerSpeed = KickerSpeed;
            this.UpsySpeed = UpsySpeed;
            this.DiverterSpeed = DiverterSpeed;
        }

        public double getSpinnerSpeed() {
            return SpinnerSpeed * SPINNER_CONVERSION;
        }
        public double getUpsySpeed() {
            return UpsySpeed;
        }

        public double getKickerSpeed() {
            return KickerSpeed;
        }
        public double getDiverterSpeed() {
            return DiverterSpeed;
        }
    }

    public enum ClimberStates {
        STOW(0.0),
        RUNG_ONE(8.0),
        RUNG_TWO(0.0),
        RUNG_THREE(0.0),
        PRESTAGE(0.0),
        PULL(2.5);

        private final double Climb;
        private final double CLIMBER_CONVERSION = 0.1256;

        private ClimberStates(double Climb) {
            this.Climb = Climb;
        }

        public double getClimb() {
            return Climb / CLIMBER_CONVERSION;
        }

    }
}