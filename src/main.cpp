#include "global.h"
#include "db/Database.h"
#include "fp/floorplanner.h"

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
    
    db::Database database;

    database.designName = design;

    database.read(blockFile, netFile, plFile);
    database.init(ratio);

    // database.recall_design();
    // exit(1);

    // fp::Floorplan fp(&database);

    fp::Floorplanner fp(&database, 0.1, "fast", true, fp_path);
    fp.Run();

    // fp.write(fp_path);
    

    // database.readBSCell(blockFile);
    // database.readBSNets(netFile);
    // database.readBSPl(plFile);

    log() << "-----------finish I/O-------------" << std::endl;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    srand (time(NULL));
    // utils::timer runtime;
    // for (int i = 0; i < 885 ; i++){

    // }
    // double pack_time = runtime.elapsed();
    // cout << "========== Pack time: " << pack_time << " s ==========\n\n\n";
    // exit(1);
    std::cout << std::boolalpha;  // set std::boolalpha to std::cout
    
    log() << "-----------start-------------" << std::endl;


    partition(argv);

    printlog("---------------------------------------------------------------------------");
    printlog("                               Terminated...                               ");
    printlog("---------------------------------------------------------------------------");

    return 0;
}