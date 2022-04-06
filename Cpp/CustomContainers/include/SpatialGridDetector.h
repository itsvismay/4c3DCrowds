// SpatialGridDetector.h
//
// Breannan Smith
// Last updated: 09/14/2015

#ifndef SPATIAL_GRID_DETECTOR_H
#define SPATIAL_GRID_DETECTOR_H
#include <Eigen/Core>
#include <set>
#include <vector>

class AABB final
{

public:

  bool overlaps( const AABB& other ) const;

  // TODO: Remove non-const accessors, replace with constructor
  inline Eigen::Vector3d& min() { return m_min; }
  inline Eigen::Vector3d& max() { return m_max; }

  inline const Eigen::Vector3d& min() const { return m_min; }
  inline const Eigen::Vector3d& max() const { return m_max; }

private:  

  Eigen::Vector3d m_min;
  Eigen::Vector3d m_max;

};

namespace SpatialGridDetector
{
  void getPotentialOverlaps( const std::vector<AABB>& aabbs, std::set<std::pair<unsigned,unsigned>>& overlaps );
  void getPotentialOverlapsAllPairs( const std::vector<AABB>& aabbs, std::set<std::pair<unsigned,unsigned>>& overlaps );
}

#endif