// Exact, single-thread kNN reduction. Uses the installed BSD nanoflann header.
// No OpenMP, worker pool, ROS, or retained point pointers across calls.
#include <nanoflann.hpp>
#include <array>
#include <cmath>
#include <cstddef>

struct Points {
  const double* xyz;
  std::size_t n;
  std::size_t kdtree_get_point_count() const { return n; }
  double kdtree_get_pt(std::size_t i, std::size_t d) const { return xyz[3*i+d]; }
  template<class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
};

extern "C" int knn_mean(const double* xyz, std::size_t n, int k, double* mean) noexcept {
  if (!xyz || !mean || k<2 || k>128 || n<=static_cast<std::size_t>(k)) return 1;
  try {
    Points points{xyz,n};
    using Metric=nanoflann::L2_Simple_Adaptor<double,Points>;
    nanoflann::KDTreeSingleIndexAdaptor<Metric,Points,3,std::size_t> tree(
        3,points,nanoflann::KDTreeSingleIndexAdaptorParams(16));
    tree.buildIndex();
    std::array<std::size_t,129> indices;
    std::array<double,129> distances;
    for (std::size_t i=0;i<n;++i) {
      nanoflann::KNNResultSet<double,std::size_t> result(k+1);
      result.init(indices.data(),distances.data());
      tree.findNeighbors(result,xyz+3*i,nanoflann::SearchParams());
      if (result.size()!=static_cast<std::size_t>(k+1)) return 2;
      double sum=0;
      // Skip one zero-distance self neighbor, including duplicate-point ties.
      for (int j=1;j<=k;++j) sum+=std::sqrt(distances[j]);
      mean[i]=sum/k;
    }
    return 0;
  } catch (...) { return 3; }
}
