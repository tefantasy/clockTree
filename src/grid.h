#pragma once
#include <algorithm>
#include "robin_hood.h"
#include "ctr.h"

class Grid {
public:
    Grid() : fInit(false) {}
    ~Grid() {
        if (fInit) {
            delete [] vCellTrees;
            delete [] vHorEdgeRes;
            delete [] vVerEdgeRes;
            delete [] vVisited;
        }
    }

    void initGrid(int nGridSize, int nEdgeCap);
    int getEdgeRes(const Edge & e) {
        if (e.isHorizontal())
            return vHorEdgeRes[getHorEdgeIdx(e)];
        else
            return vVerEdgeRes[getVerEdgeIdx(e)];
    }
    void decreaseEdgeRes(const Edge & e) {
        if (e.isHorizontal()) {
            vHorEdgeRes[getHorEdgeIdx(e)]--;
        } else {
            vVerEdgeRes[getVerEdgeIdx(e)]--;
        }
    }
    void increaseEdgeRes(const Edge & e) {
        if (e.isHorizontal()) {
            vHorEdgeRes[getHorEdgeIdx(e)]++;
            assert(vHorEdgeRes[getHorEdgeIdx(e)] <= nEdgeCap);
        } else {
            vVerEdgeRes[getVerEdgeIdx(e)]++;
            assert(vVerEdgeRes[getVerEdgeIdx(e)] <= nEdgeCap);
        }
    }

    [[deprecated]] bool cellHasTree(const Point & p, int treeId) {
        int pointIdx = p.y * nGridSize + p.x;
        return vCellTrees[pointIdx].find(treeId) != vCellTrees[pointIdx].end();
    }
    int * getVisitedMem() { return vVisited; }


private:
    int getHorEdgeIdx(const Edge & e) { assert(e.p1.x < nGridSize - 1); return e.p1.y * (nGridSize - 1) + e.p1.x; }
    int getVerEdgeIdx(const Edge & e) { assert(e.p1.y < nGridSize - 1); return e.p1.y * nGridSize + e.p1.x; }

    bool fInit;
    int nGridSize, nEdgeCap;
    robin_hood::unordered_set<int> * vCellTrees; // the tree ids in each cell
    int * vHorEdgeRes, * vVerEdgeRes;            // the remaining edge resources
    int * vVisited;                              // memory for marking visited cells during routing
};