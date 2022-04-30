#pragma once
#include <Eigen/Core>

class SpatialHash{

    double spacing;
    int tableSize;
    Eigen::VectorXi cellStart;
    Eigen::VectorXi cellEntries;
    Eigen::VectorXi cellEntries;


    SpatialHash(double spacing, int maxObjects){
        tableSize = 2*maxObjects;
        spacing = spacing;
        cellStart = Eigen::VectorXi::Zeros(tableSize + 1);
        cellEntries = Eigen::VectorXi::Zeros(maxObjects);
        queryIds = Eigen::VectorXi::Zeros(maxObjects);
        querySize = 0;
    }

    int hashCoords(int xi, int yi, int zi){
        int h = (ix * 92837111) ^ (iy * 689287499) ^ (iz * 83492791);
        return abs(h) % this->tableSize;
    }

    int intCoord(int coord){
        return floor(coord/this->spacing);
    }

    int hashPos(Eigen::Vector3d pos, nr) {
    return this.hashCoords(
    this.intCoord(pos[3 * nr]), 
    this.intCoord(pos[3 * nr + 1]),
    this.intCoord(pos[3 * nr + 2]));
    }


    //64 bit return value should help
    inline size_t hash(Eigen::Vector3d &p, float invGridSpacing, const int n) {

        int ix = (unsigned int)((p[0]) *invGridSpacing);
        int iy = (unsigned int)((p[1]) *invGridSpacing);
        int iz = (unsigned int)((p[2]) *invGridSpacing);
        return (unsigned int)((ix * 92837111) ^ (iy * 689287499) ^ (iz * 83492791)) % n;
    }


};

// class TupleHash : public SpatialHash
// {
//      //64 bit return value should help
//     std::tuple<int, int, int> xyzhash(Eigen::Vector3f &p) {

//         float px = p[0];
//         //get xcell
//         int xcell = floor((px - -10)/cellSize);
//         float py = p[1];
//         //get ycell
//         int ycell = floor((py - -1)/cellSize);
//         float pz = p[2];
//         //get zcell
//         int zcell = floor((pz - -10)/cellSize);

//         std::tuple<int, int, int> xyzcells{xcell, ycell, zcell};
//         return xyzcells;
//     }
// };