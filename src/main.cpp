#include "global.h"
#include "db/Database.h"
#include "fp/floorplan.h"

void signalHandler(int signum) {
    std::cout << "Signal (" << signum << ") received. Exiting...\n";

    std::exit(signum);
}

// -----------------------------------------------------------------------------

void partition(char* argv[]){
    log() << "FloorPlanning" << std::endl;

    

    // input
    string blockFile    = string(argv[1]);
    string netFile      = string(argv[2]);
    string plFile       = string(argv[3]);
    string fp_path      = string(argv[4]);
    double ratio        = stof((argv[5]));

    // required
    std::string prefix = blockFile.substr(0,blockFile.find_last_of('.'));
    std::string design = prefix.substr(prefix.find_last_of('/'));
    

    utils::timer runtime;

    db::Database database;
    database.designName = design;
    database.read(blockFile, netFile, plFile);
    database.init(ratio);

    // io time


    fp::Floorplan fp(&database, 0.9, "fast", false, fp_path);
    double io_time = runtime.elapsed();
    
    fp.Run();

    // runtime
    double exe_time = runtime.elapsed();
    log() << "========== IO time: " << io_time << " s ==========\n";
    log() << "========== Execution time: " << exe_time - io_time << " s ==========\n\n\n";

    // fp.write(fp_path);
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    srand (time(NULL));

    std::cout << std::boolalpha;  // set std::boolalpha to std::cout
    
    log() << "-----------start-------------" << std::endl;


    partition(argv);

    printlog("---------------------------------------------------------------------------");
    printlog("                               Terminated...                               ");
    printlog("---------------------------------------------------------------------------");

    return 0;
}