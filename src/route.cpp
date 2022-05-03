#include <vector>
#include <algorithm>
#include "robin_hood.h"
#include "ctr.h"
#include "ctr_man.h"

extern "C" {
#include "flute.h"
}

void CTRMan::initFlute() {
    readLUT();
}

std::vector<Edge> getConnEdges(const Point & p0, const Point & p1) {
    std::vector<Edge> vConnEdges;
    for (int x = min(p0.x, p1.x); x < max(p0.x, p1.x); x++) {
        vConnEdges.emplace_back(Point(x, p0.y), Point(x + 1, p0.y));
    }
    for (int y = min(p0.y, p1.y); y < max(p0.y, p1.y); y++) {
        vConnEdges.emplace_back(Point(p1.x, y), Point(p1.x, y + 1));
    }
    return vConnEdges;
}

void CTRMan::routeNet(int cluster) {
    int nNetPins = vClusterSizes[cluster] + 1; // include the tap
    int * vPinx = new int[100 * nNetPins];
    int * vPiny = new int[100 * nNetPins];

    for (int i = 0; i < vClusterSizes[cluster]; i++) {
        int iPin = vClusters[cluster][i];
        vPinx[i] = vPins[iPin].x;
        vPiny[i] = vPins[iPin].y;
    }
    vPinx[vClusterSizes[cluster]] = vTaps[cluster].x;
    vPiny[vClusterSizes[cluster]] = vTaps[cluster].y;

    Tree fluteTree = flute(nNetPins, vPinx, vPiny, ACCURACY);
    // printtree(fluteTree);
    assert(nNetPins == fluteTree.deg);

    auto insertEdge = [&](const Edge & e, const FluteConn & conn){
        auto pairRes = vClusterEdges[cluster].insert(e);
        if (!pairRes.second)
            printf("Warning: detected duplicate edge in the flute tree\n");
        grid.decreaseEdgeRes(e);
        
        // record possible overflow edges
        if (grid.getEdgeRes(e) < 0)
            sOverflowEdges.insert(e);

        // record flute connection
        if (edgeToConn.find(e) == edgeToConn.end())
            edgeToConn.insert({e, std::vector<FluteConn>()});
        edgeToConn.at(e).push_back(conn);
    };

    for (int i = 0; i < 2 * nNetPins - 2; i++) {
        const Branch & branch0 = fluteTree.branch[i];
        const Branch & branch1 = fluteTree.branch[branch0.n];

        Point p0(branch0.x, branch0.y), p1(branch1.x, branch1.y);
        if (p0 == p1) continue;
        // assume that there is no duplicate flute connection (Branch)
        FluteConn conn = {cluster, p0, p1};

        // add edges from p0 to p1
        std::vector<Edge> vConnEdges = getConnEdges(p0, p1);
        for (const Edge & e : vConnEdges)
            insertEdge(e, conn);
        // for (int x = min(p0.x, p1.x); x < max(p0.x, p1.x); x++) {
        //     Edge e(Point(x, p0.y), Point(x + 1, p0.y));
        //     insertEdge(e, conn);
        // }
        // for (int y = min(p0.y, p1.y); y < max(p0.y, p1.y); y++) {
        //     Edge e(Point(p1.x, y), Point(p1.x, y + 1));
        //     insertEdge(e, conn);
        // }
    }

    free(fluteTree.branch);
    delete [] vPinx;
    delete [] vPiny;
}

void CTRMan::reRoute() {
    if (sOverflowEdges.size() == 0)
        return;

    printf("Performing reroute...\n");
    robin_hood::unordered_set<FluteConn, flute_conn_hash> sOverflowConns;
    printf("Overflowed edges: ");
    for (const Edge & e : sOverflowEdges) {
        printf("(%d, %d)-(%d, %d)    ", e.p1.x, e.p1.y, e.p2.x, e.p2.y);
        for (const FluteConn & conn : edgeToConn.at(e))
            sOverflowConns.insert(conn);
    }
    printf("\n");

    printf("Overflowed flute connections: \n");
    for (auto [cluster, p0, p1] : sOverflowConns) {
        printf("(%d, %d)-(%d, %d)    ", p0.x, p0.y, p1.x, p1.y);
    }
    printf("\n");

    // rip up all edges of these connections
    for (auto [cluster, p0, p1] : sOverflowConns) {
        std::vector<Edge> vConnEdges = getConnEdges(p0, p1);
        for (const Edge & e : vConnEdges) {
            auto ret = vClusterEdges[cluster].erase(e);
            if (!ret)
                printf("Warning: duplicate ripping up an edge\n");
            grid.increaseEdgeRes(e);
        }
    }

    for (const Edge & e : sOverflowEdges)
        assert(grid.getEdgeRes(e) >= 0);

    // reroute overflowed connections
    std::vector<FluteConn> vOverflowConns(sOverflowConns.begin(), sOverflowConns.end());
    // sort in increasing L1-dist order
    std::sort(vOverflowConns.begin(), vOverflowConns.end(), 
              [](const FluteConn & conn0, const FluteConn & conn1) {
                  auto [cluster0, p00, p01] = conn0;
                  auto [cluster1, p10, p11] = conn1;
                  return distL1(p00, p01) < distL1(p10, p11);
              });

}

void CTRMan::route() {
    printf("Performing flute-based net routing...\n");
    
    initRoute();
    for (int i = 0; i < nTaps; i++)
        routeNet(i);
    reRoute();

    printf("Flute-based net routing complete!\n");
}