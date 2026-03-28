package artery.application;

simple TrajektoriAppKF4D
{
    parameters:
        double logInterval @unit(s) = default(0.1s);
        double staticSpeedThreshold = default(0.5);
        @display("i=block/app");
        @class(TrajektoriAppKF4D);
}