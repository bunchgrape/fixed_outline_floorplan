#include "Database.h"


using namespace db;

/***** Database *****/
Database::Database() {
    _buffer = new char[_bufferCapacity];
}

Database::~Database() {
    delete[] _buffer;
    _buffer = nullptr;
    // for regions.push_back(new Region("default"));
} //END MODULE

//---------------------------------------------------------------------

Cell* Database::addCell(const string& name) {
    Cell* cell = getCell(name);
    if (cell) {
        printlog("cell re-defined: %s", name.c_str());
        return cell;
    }
    cell = new Cell(name);
    name_cells.emplace(name, cell);
    cells.push_back(cell);
    return cell;
} //END MODULE

//---------------------------------------------------------------------

Macro* Database::addMacro(const string& name) {
    Macro* macro = getMacro(name);
    if (macro) {
        printlog("macro re-defined: %s", name.c_str());
        return macro;
    }
    macro = new Macro(name);
    name_macros.emplace(name, macro);
    macros.push_back(macro);
    return macro;
} //END MODULE

//---------------------------------------------------------------------

Net* Database::addNet(const string& name) {
    // Net* net = getNet(name);
    // if (net) {
    //     printlog("Net re-defined: %s", name.c_str());
    //     return net;
    // }
    Net* net = new Net(name);
    name_nets[name] = net;
    nets.push_back(net);
    return net;
} //END MODULE

//---------------------------------------------------------------------

Cell* Database::getCell(const string& name) {
    unordered_map<string, Cell*>::iterator mi = name_cells.find(name);
    if (mi == name_cells.end()) {
        return nullptr;
    }
    return mi->second;
} //END MODULE

//---------------------------------------------------------------------

Macro* Database::getMacro(const string& name) {
    unordered_map<string, Macro*>::iterator mi = name_macros.find(name);
    if (mi == name_macros.end()) {
        return nullptr;
    }
    return mi->second;
} //END MODULE

//---------------------------------------------------------------------

Net* Database::getNet(const string& name) {
    unordered_map<string, Net*>::iterator mi = name_nets.find(name);
    if (mi == name_nets.end()) {
        return nullptr;
    }
    return mi->second;
} //END MODULE

//---------------------------------------------------------------------
