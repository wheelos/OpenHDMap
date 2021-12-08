#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


static const std::size_t BUFFER_SIZE = 20;

using Point = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<Point>;
using PointCloudPtr = pcl::PointCloud<Point>::Ptr;
using PointCloudConstPtr = const pcl::PointCloud<Point>::Ptr;


class AsyncBuffer {
 public:
  explicit AsyncBuffer(const std::vector<std::string>& file_paths) : 
      total_size_(file_paths.size()), file_paths_(file_paths) {}

  void Init();

  PointCloudPtr Get();

  bool IsEnd();

 private:
  std::size_t Size();

  bool Empty();

  std::mutex mutex_;
  uint32_t index_ = 0;
  std::size_t total_size_;

  std::queue<PointCloudPtr> buffer_;
  std::vector<std::string> file_paths_;
};
