#include "robin_hood.h"
#include "grid.h"

void Grid::initGrid(int nGridSize, int nEdgeCap) {
    this->nGridSize = nGridSize, this->nEdgeCap = nEdgeCap;

    vCellTrees = new robin_hood::unordered_set<int>[nGridSize * nGridSize];
    vHorEdgeRes = new int[(nGridSize - 1) * nGridSize]; // use the left point for indexing
    vVerEdgeRes = new int[nGridSize * (nGridSize - 1)]; // use the bottom point for indexing
    vVisited = new int[nGridSize * nGridSize];

    std::fill_n(vHorEdgeRes, (nGridSize - 1) * nGridSize, nEdgeCap);
    std::fill_n(vVerEdgeRes, (nGridSize - 1) * nGridSize, nEdgeCap);

    fInit = true;
}

void Grid::freeGrid() {
    if (fInit) {
        delete [] vCellTrees;
        delete [] vHorEdgeRes;
        delete [] vVerEdgeRes;
        delete [] vVisited;
        fInit = false;
    }
}