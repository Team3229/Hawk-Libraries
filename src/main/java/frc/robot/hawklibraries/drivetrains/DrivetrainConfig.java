package frc.robot.hawklibraries.drivetrains;

public abstract class DrivetrainConfig {
    
    protected double wheelDiameter;
    protected double gearRatio;
    protected double maxSpeed;
    protected double maxAccel;
    protected double maxTurnRate;
    protected double robotWidth;
    protected double robotLength;

    protected DrivetrainConfig(Builder<?> builder) {
        
        this.wheelDiameter = builder.wheelDiameter;
        this.gearRatio = builder.gearRatio;
        this.maxSpeed = builder.maxSpeed;
        this.maxAccel = builder.maxAccel;
        this.maxTurnRate = builder.maxTurnRate;
        this.robotWidth = builder.robotWidth;
        this.robotLength = builder.robotLength;

    }

    public double getWheelDiameter() {return this.wheelDiameter;}
    public double getGearRatio() {return this.gearRatio;}
    public double getMaxSpeed() {return this.maxSpeed;}
    public double getMaxAccel() {return this.maxAccel;}
    public double getmaxTurnRate() {return this.maxTurnRate;}
    public double getRobotWidth() {return this.robotWidth;}
    public double getRobotLength() {return this.robotLength;}

    public static abstract class Builder<T extends Builder<T>> {
        
        protected double wheelDiameter = 0.25;
        protected double gearRatio = 1.0;
        protected double maxSpeed = 3.0;
        protected double maxAccel = 1.0;
        protected double maxTurnRate = 1.0;
        protected double robotWidth = 1.0;
        protected double robotLength = 1.0;

        public T wheelDiameter(double wheelDiameter) {
            this.wheelDiameter = wheelDiameter;
            return self();
        }

        public T gearRatio(double gearRatio) {
            this.gearRatio = gearRatio;
            return self();
        }

        public T maxSpeed(double maxSpeed) {
            this.maxSpeed = maxSpeed;
            return self();
        }

        public T maxAccel(double maxAccel) {
            this.maxAccel = maxAccel;
            return self();
        }

        public T maxTurnRate(double maxTurnRate) {
            this.maxTurnRate = maxTurnRate;
            return self();
        }

        public T robotWidth(double robotWidth) {
            this.robotWidth = robotWidth;
            return self();
        }

        public T robotLength(double robotLength) {
            this.robotLength = robotLength;
            return self();
        }

        protected abstract T self();

        public abstract DrivetrainConfig build();
    }
}