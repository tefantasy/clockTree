#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <iostream>
#include <vector>
#include <tuple>
#include <algorithm>
#include "lemon/list_graph.h"
#include "lemon/matching.h"
#include "robin_hood.h"
#include "ctr.h"
#include "ctr_man.h"

// get the pairs of trees to be combined
std::vector< std::pair<int, int> >
CTRMan::performMatching() {
    using Graph = lemon::ListGraph;
    using GNode = Graph::Node;
    using GEdge = Graph::Edge;
    using WeightMap = Graph::EdgeMap<int>;
    using MaxWeightedMatching = lemon::MaxWeightedMatching<Graph, WeightMap>;

    const TreeList & vLevelTrees = vTrees.back();
    std::vector< std::pair<int, int> > vMatches;

    int nVertices = vLevelTrees.size();
    int maxCost = (nGridSize + 1) * 4;

    Graph g;
    std::vector<GNode> vNodes;
    vNodes.reserve(nVertices);
    for (int i = 0; i < nVertices; i++)
        vNodes.push_back(g.addNode());
    
    GEdge e;
    WeightMap weights(g);
    for (int i = 0; i < nVertices; i++)
        for (int j = i + 1; j < nVertices; j++)
            if (vLevelTrees[i]->nPins() + vLevelTrees[j]->nPins() <= nMaxTapLoad && 
                (vLevelTrees[i]->cluster == -1 && vLevelTrees[j]->cluster == -1) || 
                (vLevelTrees[i]->cluster == vLevelTrees[j]->cluster)) {
                int minDist = distL1(vLevelTrees[i]->root, vLevelTrees[j]->root) 
                              + abs(vLevelTrees[i]->delay - vLevelTrees[j]->delay);
                if (minDist % 2 == 1) continue; // if odd, can never achieve zero-skew by combining this two trees (need checking)
                int cost = maxCost - minDist; // for min weight max cardinality matching
                assert(cost > 0);
                e = g.addEdge(vNodes[i], vNodes[j]);
                weights[e] = cost;
            }
    std::cout << "Matching " << nVertices << " vertices ..." << std::endl;

    MaxWeightedMatching mwm(g, weights);
    mwm.run();

    if (mwm.matchingSize() == 0) {
        std::cout << "Cannot find any matched pair, failed to construct zero-skew." << std::endl;
        vMatches.clear();
        return vMatches;
    }

    std::cout << "Found matching with size " << mwm.matchingSize() 
              << ", and total weight " << mwm.matchingWeight() << std::endl;

    GNode u, v;
    for (int i = 0; i < nVertices; i++) {
        u = vNodes[i];
        v = mwm.mate(u);
        if (v != lemon::INVALID && g.id(u) < g.id(v)) {
            vMatches.push_back({g.id(u), g.id(v)});
            std::cout << g.id(u) << " <-----> " << g.id(v) << std::endl;
        }
    }
    
    return vMatches; // records pairs of tree indices in the last level of vTrees
}

// Combine two zero-skew trees and form a new zero-skew tree. 
// Route between two old roots and get the new root location. 
bool CTRMan::combineTrees(TreeTopoNode * p0, TreeTopoNode * p1, int oldLevel) {
    const Point & root0 = p0->root;
    const Point & root1 = p1->root;
    int delay0 = p0->delay, delay1 = p1->delay;

    int skew = abs(delay0 - delay1);
    int minDist = distL1(root0, root1) + skew;

    // gather all nodes in the two trees to visited
    int * vVisited = grid.getVisitedMem();
    memset(vVisited, 0, nGridSize * nGridSize * sizeof(int));
    for (const auto & e : p0->sWires) {
        vVisited[e.p1.y * nGridSize + e.p1.x] = 1;
        vVisited[e.p2.y * nGridSize + e.p2.x] = 1;
    }
    for (const auto & e : p1->sWires) {
        vVisited[e.p1.y * nGridSize + e.p1.x] = 1;
        vVisited[e.p2.y * nGridSize + e.p2.x] = 1;
    }
    if (p0->sWires.size() > 0)
        assert(vVisited[root0.y * nGridSize + root0.x] == 1);
    if (p1->sWires.size() > 0)
        assert(vVisited[root1.y * nGridSize + root1.x] == 1);
    // also gather other trees of the same cluster to visited
    assert(p0->cluster == p1->cluster);
    assert(vClusterTrees[p0->cluster].find(p0) != vClusterTrees[p0->cluster].end());
    assert(vClusterTrees[p0->cluster].find(p1) != vClusterTrees[p0->cluster].end());
    for (const TreeTopoNode * pTree : vClusterTrees[p0->cluster]) {
        if (pTree == p0 || pTree == p1) continue;
        for (const auto & e : pTree->sWires) {
            vVisited[e.p1.y * nGridSize + e.p1.x] = 1;
            vVisited[e.p2.y * nGridSize + e.p2.x] = 1;
        }
    }
    // the two roots need to be visited during routing, so unmark them
    vVisited[root0.y * nGridSize + root0.x] = 0;
    vVisited[root1.y * nGridSize + root1.x] = 0;

    // perform routing between two roots
    std::vector<Point> vPath = routeWithLenRange(root0, root1, vVisited, minDist, minDist * 2);
    if (vPath.size() == 0) {
        // routing failed
        printf("Warning: Route (%d, %d)[delay=%d] to (%d, %d)[delay=%d] failed.\n", 
               root0.x, root0.y, delay0, root1.x, root1.y, delay1);
        return false;
    }

    // success, then create a new TreeTopoNode
    TreeTopoNode * pTreeNew = addTree(p0, p1, oldLevel + 1);

    // update vClusterTrees
    vClusterTrees[p0->cluster].erase(p0);
    vClusterTrees[p0->cluster].erase(p1);
    vClusterTrees[p0->cluster].insert(pTreeNew);
    
    // find and assign root, delay, and new wires
    int nPathLen = vPath.size();
    int nPathLen0, nPathLen1; // the length from root0/1 to the new root
    if (delay0 > delay1) {
        nPathLen0 = (nPathLen - skew) / 2;
        nPathLen1 = (nPathLen - skew) / 2 + skew;
    } else {
        nPathLen0 = (nPathLen - skew) / 2 + skew;
        nPathLen1 = (nPathLen - skew) / 2;
    }
    assert(nPathLen0 + nPathLen1 == nPathLen);

    vPath.push_back(root1);
    pTreeNew->root.x = vPath[nPathLen0].x, pTreeNew->root.y = vPath[nPathLen0].y;
    printf("  Found new root (%d, %d) combining tree (%d, %d)[delay=%d] and (%d, %d)[delay=%d], #pins = %d. \n",
           pTreeNew->root.x, pTreeNew->root.y, root0.x, root0.y, delay0, root1.x, root1.y, delay1, pTreeNew->nPins());

    printf("    Path: ");
    for (const auto & p : vPath) {
        printf("(%d, %d) ", p.x, p.y);
    }
    printf("\n");

    assert(delay0 + nPathLen0 == delay1 + nPathLen1);
    pTreeNew->delay = delay0 + nPathLen0;

    // add new wires into the tree, and decrease edge resources in the grid
    for (int i = 0; i < nPathLen; i++) {
        pTreeNew->addWire(vPath[i], vPath[i + 1]);
        grid.decreaseEdgeRes(Edge(vPath[i], vPath[i + 1]));
        vClusterEdges[pTreeNew->cluster].insert(Edge(vPath[i], vPath[i + 1]));
    }

    return true;
}



// Find a path from src to dst, whose length is in the range of [minDist, maxDist].
// If maxDist is not specified, find a path with exact length of minDist.
std::vector<Point> CTRMan::routeWithLenRange(const Point & src, const Point & dst, int * vVisited,
                                             int minDist, int maxDist) {
    if (maxDist == 0)
        maxDist = minDist;
    
    std::vector<Point> vPath;
    vVisited[src.y * nGridSize + src.x] = 1; // visit src
    bool found = routeWithLenRangeRec(src, dst, minDist, maxDist, 0, vPath, vVisited);
    if (!found) {
        vPath.clear();
        return vPath;
    }

    assert(vPath.size() >= minDist && vPath.size() <= maxDist);
    assert(vPath[0] == src);
    assert(distL1(vPath.back(), dst) == 1);
    return vPath;
}

bool CTRMan::routeWithLenRangeRec(const Point & p, const Point & dst, int minDist, int maxDist, 
                                  int lenNow, std::vector<Point> & vPath, int * vVisited) {
    // dfs based backtracking
    // expand neighbours with more resources
    if (lenNow > maxDist)
        return false;
    
    if (p == dst) {
        if (lenNow < minDist)
            return false;
        assert(lenNow >= minDist && lenNow <= maxDist);
        assert(vPath.size() == lenNow);
        return true;
    }
    
    // gather all unvisited adjacent points satisfying the edge capacity constraint
    // note, the edge resources need not to be updated during a routing because of visited marking
    std::vector< std::tuple<Point, int, int> > vNextPoints; // {p, res, l1-dist}
    if (p.x > 0) {
        Point pNext(p.x - 1, p.y);
        int edgeRes = grid.getEdgeRes(Edge(p, pNext));
        if (!vVisited[pNext.y * nGridSize + pNext.x] && edgeRes > 0 && !isForbiddenPoint(pNext))
            vNextPoints.push_back({pNext, edgeRes, distL1(pNext, dst)});
    }
    if (p.x < nGridSize - 1) {
        Point pNext(p.x + 1, p.y);
        int edgeRes = grid.getEdgeRes(Edge(p, pNext));
        if (!vVisited[pNext.y * nGridSize + pNext.x] && edgeRes > 0 && !isForbiddenPoint(pNext))
            vNextPoints.push_back({pNext, edgeRes, distL1(pNext, dst)});
    }
    if (p.y > 0) {
        Point pNext(p.x, p.y - 1);
        int edgeRes = grid.getEdgeRes(Edge(p, pNext));
        if (!vVisited[pNext.y * nGridSize + pNext.x] && edgeRes > 0 && !isForbiddenPoint(pNext))
            vNextPoints.push_back({pNext, edgeRes, distL1(pNext, dst)});
    }
    if (p.y < nGridSize - 1) {
        Point pNext(p.x, p.y + 1);
        int edgeRes = grid.getEdgeRes(Edge(p, pNext));
        if (!vVisited[pNext.y * nGridSize + pNext.x] && edgeRes > 0 && !isForbiddenPoint(pNext))
            vNextPoints.push_back({pNext, edgeRes, distL1(pNext, dst)});
    }

    // stop if no more expansion
    if (vNextPoints.size() == 0)
        return false;
    
    // sort the points in decreasing resources, break tie using increasing l1 distance
    std::sort(vNextPoints.begin(), vNextPoints.end(), 
              [](const std::tuple<Point, int, int> & p1, const std::tuple<Point, int, int> & p2) {
                  return std::get<1>(p1) > std::get<1>(p2) || 
                         (std::get<1>(p1) == std::get<1>(p2) && std::get<2>(p1) < std::get<2>(p2));
              });
    
    // recursive expansion
    for (auto [pNext, edgeRes, dist] : vNextPoints) {
        vPath.push_back(p);
        vVisited[pNext.y * nGridSize + pNext.x] = 1;
        
        int found = routeWithLenRangeRec(pNext, dst, minDist, maxDist, lenNow + 1, vPath, vVisited);
        if (found) {
            return true;
        }

        vVisited[pNext.y * nGridSize + pNext.x] = 0;
        vPath.pop_back();
    }

    return false;
}

bool CTRMan::constructLevel(int oldLevel) {
    assert(oldLevel == vTrees.size() - 1);

    int nLevelTrees = vTrees.back().size();
    robin_hood::unordered_set<int> vNotCombinedTrees;
    for (int i = 0; i < nLevelTrees; i++)
        vNotCombinedTrees.insert(i);
    
    std::vector< std::pair<int, int> > vMatch = performMatching();

    // TODO change the order of combining trees

    int nNewLevelTrees = nLevelTrees;
    for (auto [treeIdx0, treeIdx1] : vMatch) {
        assert(treeIdx0 < nLevelTrees && treeIdx1 < nLevelTrees);

        bool combined = combineTrees(vTrees[oldLevel][treeIdx0], vTrees[oldLevel][treeIdx1], oldLevel);
        if (combined) {
            auto iter0 = vNotCombinedTrees.find(treeIdx0);
            assert(iter0 != vNotCombinedTrees.end());
            vNotCombinedTrees.erase(iter0);

            auto iter1 = vNotCombinedTrees.find(treeIdx1);
            assert(iter1 != vNotCombinedTrees.end());
            vNotCombinedTrees.erase(iter1);

            nNewLevelTrees--;
            if (nNewLevelTrees == nTaps)
                break;
        } else {
            nFailedRouting++;
            if (nFailedRouting == 4)
                break;
        }
    }

    if (vNotCombinedTrees.size() == nLevelTrees) {
        assert(oldLevel == vTrees.size() - 1);
        printf("Level %d: none of the trees are combined. Abort.\n", oldLevel);
        return false;
    }

    // raise not combined trees to the next level
    assert(oldLevel == vTrees.size() - 2);
    for (int treeIdx : vNotCombinedTrees) {
        vTrees[oldLevel + 1].push_back(vTrees[oldLevel][treeIdx]);
    }
    printf("Level %d: total #trees = %d, combined %d, new level #trees = %d.\n", oldLevel, 
           nLevelTrees, nLevelTrees - (int)vNotCombinedTrees.size(), nNewLevelTrees);
    if (nFailedRouting == 4)
        return false;
    return true;
}

bool CTRMan::rgmZeroSkew() {
    // initialize data structures
    initRgmZeroSkew();

    int level = 0;
    nFailedRouting = 0;
    while (constructLevel(level++)) {
        // check if new level has #trees < nTaps
        if (vTrees.back().size() <= nTaps)
            break;
    }

    if (vTrees.back().size() > nTaps) {
        printf("RGM with zero-skew failed.\n");
        return false;
    }

    assert(vTrees.back().size() == nTaps);

    // TODO route trees to taps

    return true;
}
