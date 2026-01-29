package frc.robot.subsystems;

public class States {
    public enum IntakeStates {
        STOW(0.0, 0.0),
        INTAKE(0.0, 0.0),
        FIRE(0.0, 0.0),
        UNCLOG(0.0, 0.0);

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
        PLACE_HOLDER(0.0,0.0,0.0,0.0, 0.0);

        private final double HoodAngle, ShooterSpeed, TurretRotation, IndexerSpeed, KickerSpeed;

        private ShootdexerStates(double HoodAngle, double ShooterSpeed, double TurretRotation, double IndexerSpeed, double KickerSpeed) {
            this.HoodAngle = HoodAngle;
            this.ShooterSpeed= ShooterSpeed;
            this.TurretRotation= TurretRotation;
            this.IndexerSpeed= IndexerSpeed;
            this.KickerSpeed= KickerSpeed;
        }

        public double getHoodAngle() {
            return HoodAngle;
        }

        public double getShooterSpeed() {
            return ShooterSpeed;
        }

        public double getTurretRotation() {
            return TurretRotation;
        }

        public double getIndexerRotation() {
            return IndexerSpeed;
        }

        public double getKickerSpeed() {
            return KickerSpeed;
        }
    }

    
    public enum ClimberStates {
        RUNG_ONE(0.0),
        RUNG_TWO(0.0),
        RUNG_THREE(0.0);

    
        private final double Climb;
        private ClimberStates(double Climb){
            this.Climb = Climb;
        }

        public double getClimb() {
            return Climb;
        }

    }
}