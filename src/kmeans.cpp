#include <limits>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <cassert>
#include "ctr.h"

struct Centroid {
    Centroid(double x, double y) : x(x), y(y) {}
    double x, y;
};

// used when gathering points from a centroid
struct ClusterPoint {
    ClusterPoint() {}
    ClusterPoint(int idx, double dist) : idx(idx), dist(dist) {}
    int idx;
    double dist;
}; 

inline double l1Dist(const Point & data, const Centroid & centroid) {
    return fabs((double)data.x - centroid.x) + fabs((double)data.y - centroid.y);
}

inline int l1Dist(const Point & data, const Point & centroid) {
    return abs(data.x - centroid.x) + abs(data.y - centroid.y);
}

std::tuple< std::vector<int>, std::vector<int> > 
kMeans(const std::vector<Pin> & vData, 
       const std::vector<Tap> & vInitCentroids, int nClusterCap, int nMaxIter) {
    int k = vInitCentroids.size();
    int n = vData.size();

    if (k == 1) {
        std::vector<int> vAssignments(n, 0);
        std::vector<int> vAssignmentsCount(1, n);
        return {vAssignments, vAssignmentsCount};
    }
    
    std::vector<Centroid> vCentroids;
    vCentroids.reserve(k);
    for (const Point & centroid : vInitCentroids)
        vCentroids.emplace_back(centroid.x, centroid.y);
    
    double dist, minDist, maxDist;
    double maeOld, mae = std::numeric_limits<double>::max();
    int centroidAssign;
    std::vector<Centroid> vDistSum(k, Centroid(0, 0));
    std::vector<int> vAssignments(n, -1);
    std::vector<int> vAssignmentsCount(k, 0);
    // main kmeans loop
    for (int it = 0; it < nMaxIter; it++) {
        for (int j = 0; j < k; j++)
            vDistSum[j].x = vDistSum[j].y = 0.0, vAssignmentsCount[j] = 0;

        maeOld = mae;
        mae = 0.0;
        for (int i = 0; i < n; i++) {
            minDist = std::numeric_limits<double>::max();
            centroidAssign = -1;
            for (int j = 0; j < k; j++) {
                dist = l1Dist(vData[i], vCentroids[j]);
                if (dist < minDist) {
                    minDist = dist, centroidAssign = j;
                }
            }
            vDistSum[centroidAssign].x += vData[i].x;
            vDistSum[centroidAssign].y += vData[i].y;
            vAssignmentsCount[centroidAssign]++;
            vAssignments[i] = centroidAssign;
            mae += minDist;
        }

        for (int j = 0; j < k; j++) {
            vCentroids[j].x = vDistSum[j].x / vAssignmentsCount[j];
            vCentroids[j].y = vDistSum[j].y / vAssignmentsCount[j];
        }

        mae /= n;
        if ((maeOld - mae) / maeOld < 1e-5) {
            printf("K-means main loop: early break at iter %d\n", it);
            break;
        }
    }

    // adjust each cluster to satisfy the nClusterCap constraint
    std::vector<int> vLargeClusterInd;
    std::vector<ClusterPoint> vClusterPoints;
    while (1) {
        vLargeClusterInd.clear();
        for (int j = 0; j < k; j++)
            if (vAssignmentsCount[j] > nClusterCap)
                vLargeClusterInd.push_back(j);
        if (vLargeClusterInd.size() == 0)
            break;
        
        // select the first over-large cluster
        int cluster = vLargeClusterInd[0];
        vClusterPoints.clear();
        for (int i = 0; i < n; i++)
            if (vAssignments[i] == cluster) {
                vClusterPoints.emplace_back(i, l1Dist(vData[i], vCentroids[cluster]));
            }
        assert(vClusterPoints.size() == vAssignmentsCount[cluster]);
        
        // get vAssignmentsCount[cluster] - nClusterCap farest points
        int nToMove = vAssignmentsCount[cluster] - nClusterCap;
        std::partial_sort(
            vClusterPoints.begin(), vClusterPoints.begin() + nToMove,
            vClusterPoints.end(), 
            [](const ClusterPoint & a, const ClusterPoint & b) { return a.dist > b.dist; }
        );

        // move these points to other clusters
        for (int i = 0; i < nToMove; i++) {
            int iPoint = vClusterPoints[i].idx;
            
            int destCluster = -1;
            minDist = std::numeric_limits<double>::max();
            for (int j = 0; j < k; j++) {
                if (vAssignmentsCount[j] >= nClusterCap)
                    continue;
                dist = l1Dist(vData[iPoint], vInitCentroids[j]);
                if (dist < minDist) {
                    minDist = dist, destCluster = j;
                }
            }
            assert(destCluster != -1 && destCluster != cluster);

            vAssignments[iPoint] = destCluster;
            vAssignmentsCount[cluster]--;
            vAssignmentsCount[destCluster]++;

            printf("moved point %d from %d to %d, vAssignmentsCount[cluster] = %d, nClusterCap = %d\n", 
                   iPoint, cluster, destCluster, vAssignmentsCount[cluster], nClusterCap);
        }
        assert(vAssignmentsCount[cluster] == nClusterCap);
    }

    // part 2. post-processing that iteratively swap pairs of pins
    // recompute centroids
    for (int j = 0; j < k; j++)
        vCentroids[j].x = vCentroids[j].y = 0.0, vAssignmentsCount[j] = 0;
    for (int i = 0; i < n; i++) {
        centroidAssign = vAssignments[i];
        vCentroids[centroidAssign].x += vData[i].x;
        vCentroids[centroidAssign].y += vData[i].y;
        vAssignmentsCount[centroidAssign]++;
    }
    for (int j = 0; j < k; j++) {
        vCentroids[j].x /= vAssignmentsCount[j];
        vCentroids[j].y /= vAssignmentsCount[j];
    }

    for (int it = 0; it < n / 10; it++) {
        // find a point p1 that is farest from its centroid
        int iFarPoint = -1;
        maxDist = 0.0;
        for (int i = 0; i < n; i++) {
            dist = l1Dist(vData[i], vCentroids[vAssignments[i]]);
            if (dist > maxDist)
                maxDist = dist, iFarPoint = i;
        }
        assert(iFarPoint >= 0);

        // find another point p2, such that swapping p1 and p2
        // has largest d(p1, c1) + d(p2, c2) - d(p1, c2) - d(p2, c1)
        int iSwapPoint = -1;
        maxDist = -9999999999.0;
        for (int i = 0; i < n; i++) {
            if (vAssignments[i] == vAssignments[iFarPoint]) continue;
            dist = l1Dist(vData[iFarPoint], vCentroids[vAssignments[iFarPoint]]) + 
                   l1Dist(vData[i], vCentroids[vAssignments[i]]) -
                   l1Dist(vData[iFarPoint], vCentroids[vAssignments[i]]) -
                   l1Dist(vData[i], vCentroids[vAssignments[iFarPoint]]);
            if (dist > maxDist)
                maxDist = dist, iSwapPoint = i;
        }
        assert(iSwapPoint >= 0);
        
        // if the above value is negative, early break
        if (maxDist < 0) {
            printf("K-means postprocessing: early stopped at iter %d\n", it);
            break;
        }

        // update centroid and assignment
        int srcC = vAssignments[iFarPoint], dstC = vAssignments[iSwapPoint];
        vCentroids[srcC].x = vCentroids[srcC].x + (vData[iSwapPoint].x - vData[iFarPoint].x) / vAssignmentsCount[srcC];
        vCentroids[srcC].y = vCentroids[srcC].y + (vData[iSwapPoint].y - vData[iFarPoint].y) / vAssignmentsCount[srcC];
        vCentroids[dstC].x = vCentroids[dstC].x + (vData[iFarPoint].x - vData[iSwapPoint].x) / vAssignmentsCount[dstC];
        vCentroids[dstC].y = vCentroids[dstC].y + (vData[iFarPoint].y - vData[iSwapPoint].y) / vAssignmentsCount[dstC];
        vAssignments[iFarPoint] = dstC;
        vAssignments[iSwapPoint] = srcC;

        printf("swapped point %d (cluster %d) with point %d (cluster %d)\n", iFarPoint, srcC, iSwapPoint, dstC);
    }


    printf("Number of points in each cluster (cap=%d): ", nClusterCap);
    for (int j = 0; j < k; j++)
        printf("%d ", vAssignmentsCount[j]);
    printf("\n");

    return {vAssignments, vAssignmentsCount};
}
