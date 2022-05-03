#include <cstdio>
#include <cassert>
#include <fstream>
#include <sstream>
#include <string>
#include "ctr.h"
#include "ctr_man.h"
#include "robin_hood.h"

void CTRMan::read(const char * filePath) {
    std::ifstream fin(filePath);
    if (fin.fail()) {
        printf("Cannot open input file at: %s.\n", filePath);
        exit(1);
    }

    std::string sLine, sToken;
    int v1, v2, v3;
    while (std::getline(fin, sLine)) {
        std::istringstream sin(sLine);
        if (!(sin >> sToken))
            continue;
        
        if (sToken == "PIN") {
            sin >> v1 >> v2 >> v3;
            vPins[v1].x = v2, vPins[v1].y = v3;
            printf("Read PIN %d: %d %d\n", v1, vPins[v1].x, vPins[v1].y);
        } else if (sToken == "TAP") {
            sin >> v1 >> v2 >> v3;
            vTaps[v1].x = v2, vTaps[v1].y = v3;
            printf("Read TAP %d: %d %d\n", v1, vTaps[v1].x, vTaps[v1].y);
        } else if (sToken == "MAX_RUNTIME") {
            sin >> nMaxTime;
        } else if (sToken == "MAX_LOAD") {
            sin >> nMaxTapLoad;
        } else if (sToken == "GRID_SIZE") {
            sin >> nGridSize;
            // TODO construct grid data structure
        } else if (sToken == "CAPACITY") {
            sin >> nEdgeCap;
        } else if (sToken == "PINS") {
            sin >> nPins;
            vPins = std::vector<Pin>(nPins, Pin());
        } else if (sToken == "TAPS") {
            sin >> nTaps;
            vTaps = std::vector<Tap>(nTaps, Tap());
        } 
    }
}

void CTRMan::write(const char * filePath) {
    std::ofstream fout(filePath);

    for (int i = 0; i < nTaps; i++) {
        fout << "TAP " << i << "\n";
        fout << "PINS " << vClusterSizes[i] << "\n";
        for (int j = 0; j < vClusters[i].size(); j++)
            fout << "PIN " << vClusters[i][j] << "\n";
        
        // TODO output routing edges
        if (i < nTaps) {
            fout << "ROUTING " << vClusterEdges[i].size() << "\n";
            for (const Edge & e : vClusterEdges[i]) {
                fout << "EDGE " << e.p1.x << " " << e.p1.y << " "
                                << e.p2.x << " " << e.p2.y << " " << "\n";
            }
        }
        else
            fout << "ROUTING " << "0" << "\n";
    }
}

void CTRMan::cluster() {

    std::tie(vPinAssign, vClusterSizes) = kMeans(vPins, vTaps, nMaxTapLoad, 50);
    assert(vPinAssign.size() == nPins);
    assert(vClusterSizes.size() == nTaps);
    
    // construct pin list for each tap
    vClusters = std::vector< std::vector<int> >(nTaps);
    for (int i = 0; i < nPins; i++)
        vClusters[vPinAssign[i]].push_back(i);
}

void CTRMan::initTrees() {
    vTrees = std::vector<TreeList>(1);

    for (int i = 0; i < vPins.size(); i++) {
        const Pin & pin = vPins[i];
        TreeTopoNode * pTree = addTree(pin, i, 0);
        vClusterTrees[vPinAssign[i]].insert(pTree);
    }
}

void CTRMan::freeTrees() {
    robin_hood::unordered_set<TreeTopoNode *> sDeleted;
    for (auto & vLevelTrees : vTrees)
        for (TreeTopoNode * pTree : vLevelTrees) {
            if (sDeleted.find(pTree) == sDeleted.end()) {
                delete pTree;
                sDeleted.insert(pTree);
            }
        }
}

void CTRMan::initForbiddenPoints() {
    sForbiddenPoints.clear();

    for (const Tap & p : vTaps) {
        sForbiddenPoints.insert(Point(p.x, p.y));
        // add the surrounding 8 points into the forbidden points set
        if (p.x > 0)
            sForbiddenPoints.insert(Point(p.x - 1, p.y));
        if (p.x > 0 && p.y < nGridSize - 1)
            sForbiddenPoints.insert(Point(p.x - 1, p.y + 1));
        if (p.y < nGridSize - 1)
            sForbiddenPoints.insert(Point(p.x, p.y + 1));
        if (p.x < nGridSize - 1 && p.y < nGridSize - 1)
            sForbiddenPoints.insert(Point(p.x + 1, p.y + 1));
        if (p.x < nGridSize - 1)
            sForbiddenPoints.insert(Point(p.x + 1, p.y));
        if (p.x < nGridSize - 1 && p.y > 0)
            sForbiddenPoints.insert(Point(p.x + 1, p.y - 1));
        if (p.y > 0)
            sForbiddenPoints.insert(Point(p.x, p.y - 1));
        if (p.x > 0 && p.y > 0)
            sForbiddenPoints.insert(Point(p.x - 1, p.y - 1));
    }
}

TreeTopoNode * CTRMan::addTree(const Pin & pin, int iPin, int level, bool fClusterInfo) {
    assert(level == 0);

    TreeTopoNode * pTree;
    if (fClusterInfo)
        pTree = new TreeTopoNode(pin, iPin, nTrees++, vPinAssign[iPin]);
    else
        pTree = new TreeTopoNode(pin, iPin, nTrees++);
    vTrees[level].push_back(pTree);
    return pTree;
}

TreeTopoNode * CTRMan::addTree(TreeTopoNode * pChild0, TreeTopoNode * pChild1, int level) {
    if (level >= vTrees.size()) {
        assert(level == vTrees.size());
        vTrees.emplace_back();
    }

    TreeTopoNode * pTree = new TreeTopoNode(pChild0, pChild1, nTrees++);
    vTrees[level].push_back(pTree);
    return pTree;
}
