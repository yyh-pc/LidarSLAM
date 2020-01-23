#ifndef POINT_CLOUD_STORAGE_H
#define POINT_CLOUD_STORAGE_H

#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/io/impl/octree_pointcloud_compression.hpp>  // seems to be missing in otree_pointcloud_compression.h

// This workaround is only available on Linux, as sigaction is only supported on UNIX systems.
#ifdef __linux__
  #include <signal.h>
  namespace
  {
    // Attach SIGFPE to c++ exception.
    // This allows to properly deal with division by 0 or other computation errors.
    // NOTE : See CompressedPointCloud::GetCloud() for more details about why this is needed.
    void sigfpe_handler(int signum) { throw std::logic_error("SIGFPE"); }
  }
#endif

//------------------------------------------------------------------------------
/*!
 * @brief Base class to store a PCL pointcloud under different formats.
 */
template<typename PointT>
struct PointCloudData
{
  using CloudT = pcl::PointCloud<PointT>;
  using CloudTPtr = typename CloudT::Ptr;

  virtual ~PointCloudData() = default;

  virtual void SetCloud(CloudTPtr const& cloud) = 0;  ///< Fill with new cloud.
  virtual CloudTPtr GetCloud() = 0;                   ///< Get stored cloud.
  virtual size_t GetMemorySize() = 0;                 ///< Approximate memory usage by pointcloud data.
};

//------------------------------------------------------------------------------
/*!
 * @brief Store PCL pointcloud without any compression.
 */
template<typename PointT>
struct UncompressedPointCloud final : public PointCloudData<PointT>
{
  using CloudT = pcl::PointCloud<PointT>;
  using CloudTPtr = typename CloudT::Ptr;

  UncompressedPointCloud(CloudTPtr const& cloud) : Cloud(cloud) {}
  virtual void SetCloud(CloudTPtr const& cloud) { this->Cloud = cloud; }
  virtual CloudTPtr GetCloud() { return this->Cloud; }
  virtual size_t GetMemorySize() { return sizeof(*this->Cloud) + (sizeof(PointT) * this->Cloud->size()); }

  private:
    CloudTPtr Cloud;  ///< Raw uncompressed pointcloud.
};

//------------------------------------------------------------------------------
/*!
 * @brief Compress (with small loss) pointcloud with octree, and store pointcloud as binary data.
 */
template<typename PointT>
struct CompressedPointCloud final : public PointCloudData<PointT>
{
  using CloudT = pcl::PointCloud<PointT>;
  using CloudTPtr = typename CloudT::Ptr;

  CompressedPointCloud(CloudTPtr const& cloud)
  {
    #ifdef __linux__
      // DEBUG : Attach SIGPFE to exception
      // See CompressedPointCloud::GetCloud() for more details about why this is needed.
      struct sigaction action;
      sigemptyset(&action.sa_mask);
      action.sa_flags = SA_NODEFER;
      action.sa_handler = &sigfpe_handler;
      sigaction(SIGFPE, &action, NULL);
    #endif
    // Compress pointcloud data
    this->SetCloud(cloud);
  }

  virtual void SetCloud(CloudTPtr const& cloud)
  {
    // Octree compression
    pcl::io::OctreePointCloudCompression<PointT> compression(pcl::io::MANUAL_CONFIGURATION, false,
                                                             0.001, // pointResolution
                                                             0.05,  // octreeResolution
                                                             false, // doVoxelGridDownDownSampling
                                                             100,   // iFrameRate
                                                             1,     // colorBitResolution
                                                             false  // doColorEncoding
                                                             );
    compression.encodePointCloud(cloud, this->CompressedData);
  }

  virtual CloudTPtr GetCloud()
  {
    // Decode compressed pointcloud
    CloudTPtr cloud(new CloudT);
    pcl::io::OctreePointCloudCompression<PointT> compression;
    #ifdef __linux__
      // DEBUG : If pointcloud has a little less than 2^i points, octree
      // compression encoding is wrongly done, and can lead to division by 0 in
      // decompression step.
      // See https://github.com/PointCloudLibrary/pcl/pull/3579 for more details.
      // This workaround is necessary until ROS uses PCL > 1.10.0.99 (>= 8ed756fcfaf710cd5f3051704fdd8af7b0d4bf61)
      try
      {
        compression.decodePointCloud(this->CompressedData, cloud);
      }
      catch (std::logic_error e)
      {
        std::cerr << "[ERROR] Decompression failed. Returning empty pointcloud." << std::endl;
      }
    #else
      compression.decodePointCloud(this->CompressedData, cloud);
    #endif
    // Set back compressed data read position to beginning (missing in OctreePointCloudCompression::decodePointCloud())
    this->CompressedData.seekg(0, ios::beg);
    return cloud;
  }

  virtual size_t GetMemorySize()
  {
    std::streampos current = this->CompressedData.tellp();
    this->CompressedData.seekp(0, ios::end);
    size_t size = this->CompressedData.tellp();
    this->CompressedData.seekp(current, ios::beg);
    return size;
  }

  private:
    std::stringstream CompressedData;  ///< Binary compressed pointcloud data.
};

//------------------------------------------------------------------------------
/*!
 * @brief Structure used to log pointclouds either under uncompressed/compressed format.
 */
template<typename PointT>
struct PointCloudStorage
{
  using CloudT = pcl::PointCloud<PointT>;
  using CloudTPtr = typename CloudT::Ptr;

  PointCloudStorage(CloudTPtr const& cloud, bool compress) { this->SetCloud(cloud, compress); }

  inline bool IsCompressed() const { return this->Compression; }
  inline size_t PointsSize() const { return this->Points; }
  inline size_t MemorySize() const { return this->Data->GetMemorySize(); }

  void SetCloud(CloudTPtr const& cloud, bool compress)
  {
    this->Compression = compress;
    this->Points = cloud->size();
    if (compress)
      this->Data.reset(new CompressedPointCloud<PointT>(cloud));
    else
      this->Data.reset(new UncompressedPointCloud<PointT>(cloud));
  }

  CloudTPtr GetCloud() const { return this->Data->GetCloud(); }

  private:
    bool Compression;  ///< Wether pointcloud data is compressed or not.
    size_t Points;     ///< Number of points in stored pointcloud.
    std::unique_ptr<PointCloudData<PointT>> Data;  ///< Pointcloud data.
};

#endif // POINT_CLOUD_STORAGE_H