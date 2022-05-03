#pragma once
#include <vector>
#include <cassert>
#include <tuple>

/* Classes and inline utils */

class Point {
public:
    Point() {}
    Point(int x, int y) : x(x), y(y) {}

    bool operator==(const Point & other) const { 
        return x == other.x && y == other.y;
    }

    int x, y;
};

class Pin : public Point {
public:
    Pin() : Point() {}
    Pin(int x, int y) : Point(x, y) {}
};

class Tap : public Point {
public:
    Tap() : Point() {}
    Tap(int x, int y) : Point(x, y) {}
};

inline int abs(int x) { return x >= 0 ? x : -x; }
inline int distL1(const Point & p0, const Point & p1) { return abs(p0.x - p1.x) + abs(p0.y - p1.y); }

class Edge {
public:
    Edge() {}
    Edge(const Point & p1, const Point & p2) {
        assert(distL1(p1, p2) == 1);
        if (p1.x == p2.x) {
            this->p1.x = p1.x, this->p2.x = p2.x;
            if (p1.y < p2.y)
                this->p1.y = p1.y, this->p2.y = p2.y;
            else
                this->p1.y = p2.y, this->p2.y = p1.y;
        } else {
            this->p1.y = p1.y, this->p2.y = p2.y;
            if (p1.x < p2.x)
                this->p1.x = p1.x, this->p2.x = p2.x;
            else
                this->p1.x = p2.x, this->p2.x = p1.x;
        }
    }

    bool operator==(const Edge & other) const { 
        return p1 == other.p1 && p2 == other.p2;
    }

    bool isHorizontal() const { return p1.y == p2.y; }
    bool isVertical() const { return p1.x == p2.x; }

    Point p1, p2; // p1 is on the left or bottom of p2
};

using FluteConn = std::tuple<int, Point, Point>; // {cluster, p0, p1}

/* Declarations */

std::tuple< std::vector<int>, std::vector<int> > 
kMeans(const std::vector<Pin> & vData, 
       const std::vector<Tap> & vInitCentroids, int nClusterCap, int nMaxIter);

/* Hash function for data structures */
// from boost (functional/hash):
// see http://www.boost.org/doc/libs/1_35_0/doc/html/hash/combine.html template
template <typename T>
inline void hash_combine(std::size_t &seed, const T &val) {
    seed ^= std::hash<T>()(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}
// auxiliary generic functions to create a hash value using a seed
template <typename T> inline void hash_val(std::size_t &seed, const T &val) {
    hash_combine(seed, val);
}
template <typename T, typename... Types>
inline void hash_val(std::size_t &seed, const T &val, const Types &... args) {
    hash_combine(seed, val);
    hash_val(seed, args...);
}

template <typename... Types>
inline std::size_t hash_val(const Types &... args) {
    std::size_t seed = 0;
    hash_val(seed, args...);
    return seed;
}

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &p) const {
        return hash_val(p.first, p.second);
    }
};

struct point_hash {
    std::size_t operator()(const Point & p) const {
        return hash_val(p.x, p.y);
    }
};

struct edge_hash {
    std::size_t operator()(const Edge & e) const {
        return hash_val(e.p1.x, e.p1.y, e.p2.x, e.p2.y);
    }
};

struct flute_conn_hash {
    std::size_t operator()(const FluteConn & conn) const {
        return hash_val(std::get<0>(conn), std::get<1>(conn).x, std::get<1>(conn).y, 
                        std::get<2>(conn).x, std::get<2>(conn).y);
    }
};
