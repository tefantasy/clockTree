#pragma once
#include <vector>
#include <unordered_set>
#include "ctr.h"
#include "robin_hood.h"

class TreeTopoNode {
public:
    TreeTopoNode() = delete;
    TreeTopoNode(const Pin & pin, int iPin, int id, int cluster = -1);
    TreeTopoNode(TreeTopoNode * pChild0, TreeTopoNode * pChild1, int id);

    int nPins() { return sPinInds.size(); }
    bool hasWire(const Point & p1, const Point & p2);
    void addWire(const Point & p1, const Point & p2);

    int id;
    int cluster; // -1 means that the cluster checking is deactivated
    Point root;
    int delay;
    TreeTopoNode * pChild0, * pChild1, * pPar;
    robin_hood::unordered_set<int> sPinInds;

    // each wire corresponds to an edge of the grid graph
    robin_hood::unordered_set<Edge, edge_hash> sWires;

};
