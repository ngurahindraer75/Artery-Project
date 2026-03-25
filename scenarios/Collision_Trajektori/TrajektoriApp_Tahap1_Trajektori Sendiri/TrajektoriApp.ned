package artery.application;

simple TrajektoriApp
{
    parameters:
        string logFileName = default("results/trajektori_log.csv");
        double logInterval @unit(s) = default(1s); // interval logging
        @display("i=block/app");
        @class(TrajektoriApp);
}
