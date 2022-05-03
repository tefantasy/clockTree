#include <vector>
#include "tree_topo.h"
#include "robin_hood.h"

TreeTopoNode::TreeTopoNode(const Pin & pin, int iPin, int id, int cluster) 
    : id(id), cluster(cluster), delay(0), pChild0(NULL), pChild1(NULL), pPar(NULL) {
    root.x = pin.x, root.y = pin.y;
    sPinInds.insert(iPin);
}

TreeTopoNode::TreeTopoNode(TreeTopoNode * pChild0, TreeTopoNode * pChild1, int id)
    : id(id), pChild0(pChild0), pChild1(pChild1), pPar(NULL) {
    // NOTE root, delay, and new wires are uninitialized
    if (pChild0->cluster != -1 || pChild1->cluster != -1) {
        assert(pChild0->cluster == pChild1->cluster);
        cluster = pChild0->cluster;
    }

    pChild0->pPar = this;
    pChild1->pPar = this;

    sPinInds.reserve(pChild0->nPins() + pChild1->nPins());
    sPinInds.insert(pChild0->sPinInds.begin(), pChild0->sPinInds.end());
    sPinInds.insert(pChild1->sPinInds.begin(), pChild1->sPinInds.end());

    sWires.insert(pChild0->sWires.begin(), pChild0->sWires.end());
    sWires.insert(pChild1->sWires.begin(), pChild1->sWires.end());
}

bool TreeTopoNode::hasWire(const Point & p1, const Point & p2) {
    Edge wire(p1, p2);
    if (sWires.find(wire) != sWires.end())
        return true;
    return false;
}

void TreeTopoNode::addWire(const Point & p1, const Point & p2) {
    Edge wire(p1, p2);
    assert(sWires.find(wire) == sWires.end());
    sWires.insert(wire);
}