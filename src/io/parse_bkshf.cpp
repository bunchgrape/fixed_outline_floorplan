#include "global.h"
#include "db/Database.h"

using namespace db;


class BookshelfData {
public:
    int nCells;
    int nNets;
    int nMacros;
    int nTerminals;
    
    std::string format;
    unordered_map<string, int> cellMap;

    vector<string> cellName;
    vector<int> cellSize;
    vector<char> cellType;
    vector<int> cellX;
    vector<int> cellY;
    vector<string> netName;
    vector<int> rectNums;
    vector<vector<int>> netCells;
    vector<vector<int>> rectBlocks;

    void clearData() {
        cellMap.clear();
        cellName.clear();
        netName.clear();
        netCells.clear();
    }
};

BookshelfData bsData;

//---------------------------------------------------------------------

bool isBookshelfSymbol(unsigned char c) {
    static char symbols[256] = {0};
    static bool inited = false;
    if (!inited) {
        symbols[(int)'('] = 1;
        symbols[(int)')'] = 1;
        // symbols[(int)'['] = 1;
        // symbols[(int)']'] = 1;
        symbols[(int)','] = 1;
        // symbols[(int)'.'] = 1;
        symbols[(int)':'] = 1;
        symbols[(int)';'] = 1;
        // symbols[(int)'/'] = 1;
        symbols[(int)'#'] = 1;
        symbols[(int)'{'] = 1;
        symbols[(int)'}'] = 1;
        symbols[(int)'*'] = 1;
        symbols[(int)'\"'] = 1;
        symbols[(int)'\\'] = 1;

        symbols[(int)' '] = 2;
        symbols[(int)'\t'] = 2;
        symbols[(int)'\n'] = 2;
        symbols[(int)'\r'] = 2;
        inited = true;
    }
    return symbols[(int)c] != 0;
} //END MODULE

//---------------------------------------------------------------------

bool readBSLine(istream& is, vector<string>& tokens) {
    tokens.clear();
    string line;
    while (is && tokens.empty()) {
        // read next line in
        getline(is, line);

        char token[1024] = {0};
        int lineLen = (int)line.size();
        int tokenLen = 0;
        for (int i = 0; i < lineLen; i++) {
            char c = line[i];
            if (c == '#') {
                break;
            }
            if (isBookshelfSymbol(c)) {
                if (tokenLen > 0) {
                    token[tokenLen] = (char)0;
                    tokens.push_back(string(token));
                    token[0] = (char)0;
                    tokenLen = 0;
                }
            } else {
                token[tokenLen++] = c;
                if (tokenLen > 1024) {
                    // TODO: unhandled error
                    tokens.clear();
                    return false;
                }
            }
        }
        // line finished, something else in token
        if (tokenLen > 0) {
            token[tokenLen] = (char)0;
            tokens.push_back(string(token));
            tokenLen = 0;
        }
    }
    return !tokens.empty();
} //END MODULE

//---------------------------------------------------------------------

bool Database::readNets(const std::string& file) {
    log() << "reading net" << std::endl;
    ifstream fs(file.c_str());
    if (!fs.good()) {
        printlog("cannot open file: %s", file.c_str());
        return false;
    }
    
    vector<string> tokens;
    int netID;
    while (readBSLine(fs, tokens)) {
        if (tokens[0] == "NET") {
            int netID = bsData.nNets++;
            bsData.netName.push_back(tokens[1]);
            bsData.netCells.resize(bsData.nNets);
            for (unsigned i = 2; i < tokens.size(); i++) {
                string cName = tokens[i];
                int cellID = bsData.cellMap[cName];
                bsData.netCells[netID].push_back(cellID);
            }
        }
        // else{
        //     for (unsigned i = 2; i < tokens.size(); i++) {
        //         string cName = tokens[i];
        //         int cellID = bsData.cellMap[cName];
        //         bsData.netCells[netID].push_back(cellID);
        //     }
        // }
    }

    fs.close();
    return true;
} //END MODULE

//---------------------------------------------------------------------

bool Database::readBSNets(const std::string& file) {
    log() << "reading BS net" << std::endl;
    ifstream fs(file.c_str());
    if (!fs.good()) {
        printlog("cannot open file: %s", file.c_str());
        return false;
    }

    vector<string> tokens;
    while (readBSLine(fs, tokens)) {
        if (tokens[0] == "NumNets") {
            // printlog(LOG_INFO, "#nets : %d", atoi(tokens[1].c_str()));
            int numNets = atoi(tokens[1].c_str());
            bsData.netName.resize(numNets);
            bsData.netCells.resize(numNets);
        } else if (tokens[0] == "NumPins") {
            // printlog(LOG_INFO, "#pins : %d", atoi(tokens[1].c_str()));
        } else if (tokens[0] == "NetDegree") {
            int degree = atoi(tokens[1].c_str());
            int netID = bsData.nNets++;
            for (int i = 0; i < degree; i++) {
                readBSLine(fs, tokens);
                string cName = tokens[0];
                if (bsData.cellMap.find(cName) == bsData.cellMap.end()) {
                    assert(false);
                    printlog("cell not found : %s", cName.c_str());
                    return false;
                }

                int cellID = bsData.cellMap[cName];
                bsData.netCells[netID].push_back(cellID);
            }
        }
    }

    fs.close();
    return true;
} //END MODULE

//---------------------------------------------------------------------

bool Database::readCells(const std::string& file) {
    log() << "reading cells" << std::endl;
    ifstream fs(file.c_str());
    if (!fs.good()) {
        printlog("cannot open file: %s", file.c_str());
        return false;
    }

    vector<string> tokens;
    while (readBSLine(fs, tokens)) {
        if (tokens.size() == 2) {
            int cellID = bsData.nCells++;
            string cName = tokens[0];

            bsData.cellName.push_back(cName);
            bsData.cellMap[cName] = cellID;
            bsData.cellSize.push_back(stoi(tokens[1]));
        }
    }

    fs.close();
    return true;
} //END MODULE

//---------------------------------------------------------------------

bool Database::readBSCell(const std::string& file) {
    log() << "reading BS cell" << std::endl;
    ifstream fs(file.c_str());
    if (!fs.good()) {
        printlog("cannot open file: %s", file.c_str());
        return false;
    }

    int num_terminal;
    int num_macro;
    vector<string> tokens;
    while (readBSLine(fs, tokens)) {
        if (tokens[0] == "NumHardRectilinearBlocks"){
            num_macro = stoi(tokens[1]);
        }
        if (tokens[0] == "NumTerminals"){
            num_terminal = stoi(tokens[1]);
        }
        if (tokens[1] == "hardrectilinear") {
            int cellID = bsData.nCells++;
            bsData.nMacros++;
            string cName = tokens[0];
            bsData.cellType.push_back('1'); // cell type 1: hardblock

            bsData.cellName.push_back(cName);
            bsData.cellMap[cName] = cellID;
            
            int num_vertex = stoi(tokens[2]);
            bsData.rectNums.push_back(num_vertex);
            bsData.rectBlocks.resize(bsData.nCells);
            for(int i = 3; i < 3 + 2 * num_vertex; i++) {
                bsData.rectBlocks[cellID].push_back(stoi(tokens[i]));
            }
        }
        else if (tokens[1] == "terminal") {
            int cellID = bsData.nCells++;
            bsData.nTerminals++;
            string cName = tokens[0];
            bsData.cellType.push_back('0'); // cell type 0: terminal

            bsData.cellName.push_back(cName);
            bsData.cellMap[cName] = cellID;
        }
    }
    if (num_macro != bsData.nMacros) {
        log() << "Error: macro number incorrect\n";
    }
    else {
        log() << num_macro << " blocks to be place\n";
    }
    if (num_terminal != bsData.nTerminals) {
        log() << "Error: macro number incorrect\n";
    }
    else {
        log() << num_terminal << " terminals fixed\n";
    }
    fs.close();
    return true;
} //END MODULE

//---------------------------------------------------------------------

bool Database::readBSPl(const std::string& file) {
    log() << "reading placement" << std::endl;
    ifstream fs(file.c_str());
    if (!fs.good()) {
        printlog("cannot open file: %s", file.c_str());
        return false;
    }

    vector<string> tokens;
    bsData.cellX.resize(bsData.nCells);
    bsData.cellY.resize(bsData.nCells);
    while (readBSLine(fs, tokens)) {
        unordered_map<string, int>::iterator itr = bsData.cellMap.find(tokens[0]);
        if (itr == bsData.cellMap.end()) {
            assert(false);
            printlog("cell not found: %s", tokens[0].c_str());
            return false;
        }
        int cell = itr->second;
        double x = atof(tokens[1].c_str());
        double y = atof(tokens[2].c_str());
        bsData.cellX[cell] = (int)round(x);
        bsData.cellY[cell] = (int)round(y);
    }
    fs.close();
    return true;
} //END MODULE

//---------------------------------------------------------------------

bool Database::read(const std::string& blockFile, const std::string& netFile, const std::string& plFile) {

    readBSCell(blockFile);
    readBSNets(netFile);
    readBSPl(plFile);

    this->nCells = bsData.nCells;
    this->nMacros = bsData.nMacros;
    this->nNets = bsData.nNets;

    // cells
    int t_id = 0;
    int m_id = 0;
    for (int i = 0; i < bsData.nCells; i++) {
        int cellID = i;
        char cellType = bsData.cellType[cellID];

        if (cellType == '0') {
            Cell* cell = this->addCell(bsData.cellName[cellID]);
            cell->id = t_id++;

            int pos_x = bsData.cellX[cellID];
            int pos_y = bsData.cellY[cellID];
            cell->pos->set_pos(pos_x, pos_y);
        }
        else if (cellType == '1') {
            Macro* macro = this->addMacro(bsData.cellName[cellID]);
            macro->id = m_id++;
            macro->rectNum = bsData.rectNums[cellID];
            macro->rects = bsData.rectBlocks[cellID];
            macro->init();
        }
    }
    
    // nets
    for (unsigned i = 0; i != bsData.nNets; ++i) {
        int netID = i;
        Net* net = this->addNet(bsData.netName[netID]);
        net->id = netID;

        for (unsigned j = 0; j != bsData.netCells[netID].size(); ++j) {
            int cellID = bsData.netCells[netID][j];
            char cellType = bsData.cellType[cellID];

            if (cellType == '0') {
                Cell* cell = this->getCell(bsData.cellName[cellID]);
                net->cell_list.push_back(cell);
            }
            else if (cellType == '1') {
                Macro* macro = this->getMacro(bsData.cellName[cellID]);
                net->macro_list.push_back(macro);
                net->macro_list_by_id.push_back(macro->id);
            }
        }
        this->nets_by_id.push_back(net->macro_list_by_id);
        net->init_bx();
        this->net_terminals.push_back(db::pin(net->min_x_, net->min_y_,
                                            net->max_x_, net->max_y_));
    }

    return true;
} //END MODULE

//---------------------------------------------------------------------