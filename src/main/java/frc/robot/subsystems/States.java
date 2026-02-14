package frc.robot.subsystems;

public class States {

    public enum IntakeStates {
        STOW(0.0, 0.0),
        INTAKE(0.5, 10.0),
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

    public enum ShootdexerStates {
        IDLE(-20, 0.0, 0.0),
        INTAKE(-20.0, 340.0, 1000.0),
        UNCLOG(20.0, -340.0, -1000.0),
        FIRE(0.0, 0.0, 0.8);

        private final double ShooterSpeed, SpinnerSpeed, KickerSpeed;

        private ShootdexerStates(double ShooterSpeed, double SpinnerSpeed, double KickerSpeed) {
            this.ShooterSpeed = ShooterSpeed;
            this.SpinnerSpeed = SpinnerSpeed;
            this.KickerSpeed = KickerSpeed;
        }

        public double getShooterSpeed() {
            return ShooterSpeed;
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