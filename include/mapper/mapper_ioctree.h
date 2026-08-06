/**
 * @file mapper_ioctree.h
 * @brief Incremental point-cloud map backed by i-Octree.
 */

#ifndef IOCTREE_MAPPER_H
#define IOCTREE_MAPPER_H

#include "mapper/mapper.h"

#include <Octree.h>

namespace mapping {

class IOctreeMapper : public Mapper {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Options {
    Options() {}
    double min_extent = 0.25;
    std::size_t bucket_size = 32;
    bool downsample = true;
    double max_range = 50.0;
  };

  explicit IOctreeMapper(const Options &options = Options());

  void Update(const MapperInput &input) override;
  void Reset() override;
  bool Save(const std::string &path) const override;
  bool GetMapCloud(CloudPtr &cloud) const override;

private:
  std::unique_ptr<thuni::Octree> MakeTree() const;

  Options options_;
  // The upstream export API is not const-qualified. Mapping calls Update and
  // snapshot export from the same worker thread, so mutable is safe here.
  mutable std::unique_ptr<thuni::Octree> ioctree_;
};

} // namespace mapping

#endif // IOCTREE_MAPPER_H
