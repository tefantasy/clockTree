#include <vector>
#include <queue>
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

bool CTRMan::reRoute() {
    if (sOverflowEdges.size() == 0)
        return true;

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
    
    // route connections, add edges
    printf("Performing reroute...\n");
    std::vector<FluteConn> vFailedConns;
    for (auto [cluster, p0, p1] : vOverflowConns) {
        printf("Routing connection (%d, %d)-(%d, %d)...\n", p0.x, p0.y, p1.x, p1.y);
        std::vector<Edge> vPath = bfsRoute(p0, p1);
        // printf("  routed path: ");
        // for (const Edge & e : vPath) {
        //     printf("(%d, %d)-(%d, %d) ", e.p1.x, e.p1.y, e.p2.x, e.p2.y);
        // }
        // printf("\n");
        if (vPath.size() == 0) {
            vFailedConns.push_back({cluster, p0, p1});
            printf("  failed!\n");
        }

        for (const Edge & e : vPath) {
            auto pairRes = vClusterEdges[cluster].insert(e);
            grid.decreaseEdgeRes(e);
            if (!pairRes.second)
                printf("Warning: added duplicate edge (%d, %d)-(%d, %d) in cluster %d\n", 
                       e.p1.x, e.p1.y, e.p2.x, e.p2.y, cluster);
        }
    }

    if (vFailedConns.size() == 0)
        return true;
    
    printf("-----Reroute-----\n");
    // try to use adjacent src/dst point to reroute for failed connections
    for (auto [cluster, p0, p1] : vFailedConns) {
        bool routed = false;

        robin_hood::unordered_set<Point, point_hash> sVisited0, sVisited1;
        std::queue<Point> qFrontier0, qFrontier1;
        std::vector<Edge> vPath;

        sVisited0.insert(p0); sVisited1.insert(p1);
        qFrontier0.push(p0); qFrontier1.push(p1);
        // expand p0
        while (!qFrontier0.empty()) {
            Point p = qFrontier0.front(); 
            qFrontier0.pop();
            if (p != p0) {
                printf("Reroute (%d, %d)-(%d, %d)\n", p.x, p.y, p1.x, p1.y);
                vPath = bfsRoute(p, p1);
                if (vPath.size() > 0) {
                    routed = true;
                    break;
                }
            }

            // expansion to a connected point of pNext
            if (p.x > 0) {
                Point pNext(p.x - 1, p.y);
                if (sVisited0.find(pNext) == sVisited0.end() &&
                    vClusterEdges[cluster].find(Edge(pNext, p)) != vClusterEdges[cluster].end()) {
                    sVisited0.insert(pNext);
                    qFrontier0.push(pNext);
                }
            }
            if (p.x < nGridSize - 1) {
                Point pNext(p.x + 1, p.y);
                if (sVisited0.find(pNext) == sVisited0.end() &&
                    vClusterEdges[cluster].find(Edge(pNext, p)) != vClusterEdges[cluster].end()) {
                    sVisited0.insert(pNext);
                    qFrontier0.push(pNext);
                }
            }
            if (p.y > 0) {
                Point pNext(p.x, p.y - 1);
                if (sVisited0.find(pNext) == sVisited0.end() &&
                    vClusterEdges[cluster].find(Edge(pNext, p)) != vClusterEdges[cluster].end()) {
                    sVisited0.insert(pNext);
                    qFrontier0.push(pNext);
                }
            }
            if (p.y < nGridSize - 1) {
                Point pNext(p.x, p.y + 1);
                if (sVisited0.find(pNext) == sVisited0.end() &&
                    vClusterEdges[cluster].find(Edge(pNext, p)) != vClusterEdges[cluster].end()) {
                    sVisited0.insert(pNext);
                    qFrontier0.push(pNext);
                }
            }
        }

        if (routed) {
            for (const Edge & e : vPath) {
                auto pairRes = vClusterEdges[cluster].insert(e);
                grid.decreaseEdgeRes(e);
                if (!pairRes.second)
                    printf("Warning: added duplicate edge (%d, %d)-(%d, %d) in cluster %d\n", 
                        e.p1.x, e.p1.y, e.p2.x, e.p2.y, cluster);
            }
            continue;
        }

        // expand p1
        while (!qFrontier1.empty()) {
            Point p = qFrontier1.front(); 
            qFrontier1.pop();
            if (p != p1) {
                for (const Point & pp : sVisited0) {
                    printf("Reroute (%d, %d)-(%d, %d)\n", pp.x, pp.y, p1.x, p1.y);
                    vPath = bfsRoute(pp, p);
                    if (vPath.size() > 0) {
                        routed = true;
                        break;
                    }
                }
                if (routed) break;
            }

            // expansion to a connected point of pNext
            if (p.x > 0) {
                Point pNext(p.x - 1, p.y);
                if (sVisited1.find(pNext) == sVisited1.end() &&
                    vClusterEdges[cluster].find(Edge(pNext, p)) != vClusterEdges[cluster].end()) {
                    sVisited1.insert(pNext);
                    qFrontier1.push(pNext);
                }
            }
            if (p.x < nGridSize - 1) {
                Point pNext(p.x + 1, p.y);
                if (sVisited1.find(pNext) == sVisited1.end() &&
                    vClusterEdges[cluster].find(Edge(pNext, p)) != vClusterEdges[cluster].end()) {
                    sVisited1.insert(pNext);
                    qFrontier1.push(pNext);
                }
            }
            if (p.y > 0) {
                Point pNext(p.x, p.y - 1);
                if (sVisited1.find(pNext) == sVisited1.end() &&
                    vClusterEdges[cluster].find(Edge(pNext, p)) != vClusterEdges[cluster].end()) {
                    sVisited1.insert(pNext);
                    qFrontier1.push(pNext);
                }
            }
            if (p.y < nGridSize - 1) {
                Point pNext(p.x, p.y + 1);
                if (sVisited1.find(pNext) == sVisited1.end() &&
                    vClusterEdges[cluster].find(Edge(pNext, p)) != vClusterEdges[cluster].end()) {
                    sVisited1.insert(pNext);
                    qFrontier1.push(pNext);
                }
            }
        }

        if (routed) {
            for (const Edge & e : vPath) {
                auto pairRes = vClusterEdges[cluster].insert(e);
                grid.decreaseEdgeRes(e);
                if (!pairRes.second)
                    printf("Warning: added duplicate edge (%d, %d)-(%d, %d) in cluster %d\n", 
                        e.p1.x, e.p1.y, e.p2.x, e.p2.y, cluster);
            }
        }
        return false;
    }
    return true;
}


std::vector<Edge> CTRMan::bfsRoute(const Point & src, const Point & dst) {
    // Hadlock's algorithm
    const int RIGHT = 1;
    const int LEFT = 2;
    const int UP = 3;
    const int DOWN = 4;

    std::vector<Edge> vPath;

    int * vGrid = grid.getVisitedMem(); // used for recording detour number
    memset(vGrid, -1, nGridSize * nGridSize * sizeof(int));
    int * vParent = new int[nGridSize * nGridSize];

    auto priorityCmp = [&](const Point & p0, const Point & p1){
        int priority0 = vGrid[p0.y * nGridSize + p0.x];
        int priority1 = vGrid[p1.y * nGridSize + p1.x];
        assert(priority0 != -1 && priority1 != -1);
        return priority0 > priority1;
    };
    std::priority_queue<Point, std::vector<Point>, decltype(priorityCmp)> qFrontier(priorityCmp);

    // TODO add all other edges in this cluster to visited = -2, except the dst

    vGrid[src.y * nGridSize + src.x] = 0;
    qFrontier.push(src);

    int edgeRes, distBefore, distAfter;
    while (!qFrontier.empty()) {
        Point p = qFrontier.top();
        qFrontier.pop();
        assert(p != dst);
        distBefore = distL1(p, dst);

        // expand p
        if (p.x > 0) {
            Point pNext(p.x - 1, p.y);
            edgeRes = grid.getEdgeRes(Edge(p, pNext));
            if (vGrid[pNext.y * nGridSize + pNext.x] == -1 && edgeRes > 0) {
                // calc detour number of pNext
                distAfter = distL1(pNext, dst);
                vGrid[pNext.y * nGridSize + pNext.x] = (
                    distAfter > distBefore ? 
                    vGrid[p.y * nGridSize + p.x] + 1 : vGrid[p.y * nGridSize + p.x]);
                qFrontier.push(pNext);
                vParent[pNext.y * nGridSize + pNext.x] = RIGHT;
                if (pNext == dst) break;
            }
        }
        if (p.x < nGridSize - 1) {
            Point pNext(p.x + 1, p.y);
            edgeRes = grid.getEdgeRes(Edge(p, pNext));
            if (vGrid[pNext.y * nGridSize + pNext.x] == -1 && edgeRes > 0) {
                // calc detour number of pNext
                distAfter = distL1(pNext, dst);
                vGrid[pNext.y * nGridSize + pNext.x] = (
                    distAfter > distBefore ? 
                    vGrid[p.y * nGridSize + p.x] + 1 : vGrid[p.y * nGridSize + p.x]);
                qFrontier.push(pNext);
                vParent[pNext.y * nGridSize + pNext.x] = LEFT;
                if (pNext == dst) break;
            }
        }
        if (p.y > 0) {
            Point pNext(p.x, p.y - 1);
            edgeRes = grid.getEdgeRes(Edge(p, pNext));
            if (vGrid[pNext.y * nGridSize + pNext.x] == -1 && edgeRes > 0) {
                // calc detour number of pNext
                distAfter = distL1(pNext, dst);
                vGrid[pNext.y * nGridSize + pNext.x] = (
                    distAfter > distBefore ? 
                    vGrid[p.y * nGridSize + p.x] + 1 : vGrid[p.y * nGridSize + p.x]);
                qFrontier.push(pNext);
                vParent[pNext.y * nGridSize + pNext.x] = UP;
                if (pNext == dst) break;
            }
        }
        if (p.y < nGridSize - 1) {
            Point pNext(p.x, p.y + 1);
            edgeRes = grid.getEdgeRes(Edge(p, pNext));
            if (vGrid[pNext.y * nGridSize + pNext.x] == -1 && edgeRes > 0) {
                // calc detour number of pNext
                distAfter = distL1(pNext, dst);
                vGrid[pNext.y * nGridSize + pNext.x] = (
                    distAfter > distBefore ? 
                    vGrid[p.y * nGridSize + p.x] + 1 : vGrid[p.y * nGridSize + p.x]);
                qFrontier.push(pNext);
                vParent[pNext.y * nGridSize + pNext.x] = DOWN;
                if (pNext == dst) break;
            }
        }

    }

    vPath.clear();
    if (vGrid[dst.y * nGridSize + dst.x] >= 0) {
        // traceback to collect paths
        Point p(dst.x, dst.y), pNext;
        while (p != src) {
            // printf("Tracing back (%d, %d)...\n", p.x, p.y);
            switch (vParent[p.y * nGridSize + p.x]) {
            case RIGHT:
                pNext.x = p.x + 1, pNext.y = p.y;
                break;
            case LEFT:
                pNext.x = p.x - 1, pNext.y = p.y;
                break;
            case UP:
                pNext.x = p.x, pNext.y = p.y + 1;
                break;
            case DOWN:
                pNext.x = p.x, pNext.y = p.y - 1;
                break;
            default:
                assert(0);
            }
            
            vPath.emplace_back(p, pNext);
            p.x = pNext.x, p.y = pNext.y;
        }
    }

    if (vPath.size() > 0)
        assert(distL1(src, dst) + 2 * vGrid[dst.y * nGridSize + dst.x] == vPath.size());

    delete [] vParent;
    return vPath;
}

void CTRMan::route() {
    printf("Performing flute-based net routing...\n");
    
    initRoute();
    for (int i = 0; i < nTaps; i++)
        routeNet(i);
    bool fReRoute = reRoute();
    if (!fReRoute) {
        printf("Reroute failed. Ignore capacity constraint and do again.\n");
        // perform again without reroute
        vClusterEdges = std::vector< robin_hood::unordered_set<Edge, edge_hash> >(nTaps);
        grid.freeGrid();
        grid.initGrid(nGridSize, nEdgeCap);

        for (int i = 0; i < nTaps; i++)
            routeNet(i);
    }

    printf("Flute-based net routing complete!\n");
}