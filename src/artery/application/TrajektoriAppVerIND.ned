package artery.application;

simple TrajektoriAppVerIND
{
    parameters:
        double logInterval @unit(s) = default(0.1s);
        double staticSpeedThreshold = default(0.5);
        @display("i=block/app");
        @class(TrajektoriAppVerIND);
}
