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
        LOCKED_IDLE(0.0,0.0),
        STOW_IDLE(0.0,0.0);

        private final double HoodAngle, TurretRotation;

        private ShootdexerStates(double HoodAngle, double TurretRotation) {
            this.HoodAngle = HoodAngle;
            this.TurretRotation= TurretRotation;

        }

        public double getHoodAngle() {
            return HoodAngle;
        }

       

        public double getTurretRotation() {
            return TurretRotation;
        }

    }

    public enum ClimberStates {
        STOW(0.0),
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