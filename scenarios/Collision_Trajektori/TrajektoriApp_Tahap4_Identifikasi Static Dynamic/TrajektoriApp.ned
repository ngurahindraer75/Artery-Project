package artery.application;

simple TrajektoriApp
{
    parameters:
        string logFileName = default("results/trajektori_log.csv");
        double logInterval @unit(s) = default(0.1s);
        double staticSpeedThreshold = default(0.5);
        @display("i=block/app");
        @class(TrajektoriApp);
}
