#pragma once
#include <vector>
#include <tuple>
#include "ctr.h"
#include "tree_topo.h"
#include "grid.h"
#include "robin_hood.h"

using TreeList = std::vector<TreeTopoNode *>;

class CTRMan {
public:
    CTRMan() : nTrees(0) {}
    ~CTRMan() { 
        freeTrees(); 
    }

    void read(const char * filePath);
    void write(const char * filePath);
    void cluster();

    bool rgmZeroSkew();

    void route();


private:
    void initRgmZeroSkew() {
        vClusterTrees = std::vector< robin_hood::unordered_set<TreeTopoNode *> >(nTaps);
        vClusterEdges = std::vector< robin_hood::unordered_set<Edge, edge_hash> >(nTaps);
        initTrees();
        initForbiddenPoints();
        grid.initGrid(nGridSize, nEdgeCap);
    }
    void initTrees();
    void freeTrees();
    void initForbiddenPoints();
    bool isForbiddenPoint(const Point & p) { return sForbiddenPoints.find(p) != sForbiddenPoints.end(); }
    TreeTopoNode * addTree(const Pin & pin, int iPin, int level, bool fClusterInfo = true);
    TreeTopoNode * addTree(TreeTopoNode * pChild0, TreeTopoNode * pChild1, int level);
    bool combineTrees(TreeTopoNode * p0, TreeTopoNode * p1, int oldLevel);

    std::vector<Point> routeWithLenRange(const Point & src, const Point & dst, int * vVisited,
                                         int minDist, int maxDist = 0);
    bool routeWithLenRangeRec(const Point & p, const Point & dst, int minDist, int maxDist, 
                              int lenNow, std::vector<Point> & vPath, int * vVisited);
    
    std::vector< std::pair<int, int> > performMatching();
    bool constructLevel(int oldLevel);

    void initRoute() {
        vClusterEdges = std::vector< robin_hood::unordered_set<Edge, edge_hash> >(nTaps);
        initFlute();
        grid.initGrid(nGridSize, nEdgeCap);
    }
    void initFlute();
    void routeNet(int cluster);
    void reRoute();


    int nMaxTime;
    int nMaxTapLoad;
    int nGridSize;
    int nEdgeCap;
    int nPins;
    int nTaps;

    int nTrees;

    int nFailedRouting;

    std::vector<Pin> vPins;
    std::vector<Tap> vTaps;

    std::vector<int> vPinAssign; // the index of the tap assigned to each pin
    std::vector<int> vClusterSizes; // the number of pins for each tap
    std::vector< std::vector<int> > vClusters; // the pin indices for each tap
    std::vector<TreeList> vTrees; // the tree root nodes for each level
    std::vector< robin_hood::unordered_set<TreeTopoNode *> > vClusterTrees; // the largest trees in each cluster
    std::vector< robin_hood::unordered_set<Edge, edge_hash> > vClusterEdges; // wires in each cluster

    // protected points when performing routing during tree combinings
    robin_hood::unordered_set<Point, point_hash> sForbiddenPoints;

    robin_hood::unordered_set<Edge, edge_hash> sOverflowEdges;
    robin_hood::unordered_map<Edge, std::vector<FluteConn>, edge_hash> edgeToConn; // flute connections (Branch) that pass an edge

    Grid grid;
};