package artery.application;

// Registering the TrajektoriAppKF6D class to OMNeT++ simulation engine
simple TrajektoriAppKF6D {
    parameters:
        double logInterval @unit(s) = default(0.1s);
        double staticSpeedThreshold = default(0.5);
        
        @display("i=block/app");
        @class(TrajektoriAppKF6D);
}
