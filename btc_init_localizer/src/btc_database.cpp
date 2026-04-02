#include "btc_init_localizer/btc_database.hpp"

#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace btc_init_localizer {
namespace {

constexpr uint32_t kMagic = 0x31535442;  // "BTS1"
constexpr uint32_t kVersion = 1;

bool writeRaw(std::ostream& os, const void* data, size_t size) {
  os.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
  return static_cast<bool>(os);
}

bool readRaw(std::istream& is, void* data, size_t size) {
  is.read(reinterpret_cast<char*>(data), static_cast<std::streamsize>(size));
  return static_cast<bool>(is);
}

template <typename T>
bool writePod(std::ostream& os, const T& value) {
  return writeRaw(os, &value, sizeof(T));
}

template <typename T>
bool readPod(std::istream& is, T* value) {
  return readRaw(is, value, sizeof(T));
}

bool writeString(std::ostream& os, const std::string& value) {
  const uint32_t len = static_cast<uint32_t>(value.size());
  if (!writePod(os, len)) {
    return false;
  }
  return writeRaw(os, value.data(), len);
}

bool readString(std::istream& is, std::string* value) {
  uint32_t len = 0;
  if (!readPod(is, &len)) {
    return false;
  }
  value->assign(len, '\0');
  if (len == 0) {
    return true;
  }
  return readRaw(is, &(*value)[0], len);
}

bool writeVec3(std::ostream& os, const Eigen::Vector3d& v) {
  for (int i = 0; i < 3; ++i) {
    if (!writePod(os, v(i))) {
      return false;
    }
  }
  return true;
}

bool readVec3(std::istream& is, Eigen::Vector3d* v) {
  for (int i = 0; i < 3; ++i) {
    if (!readPod(is, &(*v)(i))) {
      return false;
    }
  }
  return true;
}

bool writeBinaryDescriptor(std::ostream& os, const BinaryDescriptor& descriptor) {
  const uint32_t summary = static_cast<uint32_t>(descriptor.summary_);
  if (!writePod(os, summary)) {
    return false;
  }
  if (!writeVec3(os, descriptor.location_)) {
    return false;
  }

  const uint32_t bit_count = static_cast<uint32_t>(descriptor.occupy_array_.size());
  if (!writePod(os, bit_count)) {
    return false;
  }

  const uint32_t byte_count = (bit_count + 7U) / 8U;
  std::vector<uint8_t> bytes(byte_count, 0);
  for (uint32_t i = 0; i < bit_count; ++i) {
    if (descriptor.occupy_array_[i]) {
      bytes[i / 8U] |= static_cast<uint8_t>(1U << (i % 8U));
    }
  }

  if (byte_count == 0U) {
    return true;
  }
  return writeRaw(os, bytes.data(), byte_count);
}

bool readBinaryDescriptor(std::istream& is, BinaryDescriptor* descriptor) {
  uint32_t summary = 0;
  if (!readPod(is, &summary)) {
    return false;
  }
  descriptor->summary_ = static_cast<unsigned char>(summary);

  if (!readVec3(is, &descriptor->location_)) {
    return false;
  }

  uint32_t bit_count = 0;
  if (!readPod(is, &bit_count)) {
    return false;
  }

  const uint32_t byte_count = (bit_count + 7U) / 8U;
  std::vector<uint8_t> bytes(byte_count, 0);
  if (byte_count > 0U && !readRaw(is, bytes.data(), byte_count)) {
    return false;
  }

  descriptor->occupy_array_.assign(bit_count, false);
  for (uint32_t i = 0; i < bit_count; ++i) {
    const bool bit = (bytes[i / 8U] >> (i % 8U)) & 1U;
    descriptor->occupy_array_[i] = bit;
  }

  return true;
}

bool writeSTD(std::ostream& os, const STD& std_item) {
  if (!writeVec3(os, std_item.triangle_)) {
    return false;
  }
  if (!writeVec3(os, std_item.angle_)) {
    return false;
  }
  if (!writeVec3(os, std_item.center_)) {
    return false;
  }

  const int32_t frame_number = static_cast<int32_t>(std_item.frame_number_);
  if (!writePod(os, frame_number)) {
    return false;
  }

  if (!writeBinaryDescriptor(os, std_item.binary_A_)) {
    return false;
  }
  if (!writeBinaryDescriptor(os, std_item.binary_B_)) {
    return false;
  }
  if (!writeBinaryDescriptor(os, std_item.binary_C_)) {
    return false;
  }

  return true;
}

bool readSTD(std::istream& is, STD* std_item) {
  if (!readVec3(is, &std_item->triangle_)) {
    return false;
  }
  if (!readVec3(is, &std_item->angle_)) {
    return false;
  }
  if (!readVec3(is, &std_item->center_)) {
    return false;
  }

  int32_t frame_number = 0;
  if (!readPod(is, &frame_number)) {
    return false;
  }
  std_item->frame_number_ = frame_number;

  if (!readBinaryDescriptor(is, &std_item->binary_A_)) {
    return false;
  }
  if (!readBinaryDescriptor(is, &std_item->binary_B_)) {
    return false;
  }
  if (!readBinaryDescriptor(is, &std_item->binary_C_)) {
    return false;
  }

  return true;
}

}  // namespace

bool saveDatabaseEntry(const std::string& root_dir, const DatabaseEntry& entry) {
  const boost::filesystem::path root(root_dir);
  const boost::filesystem::path entries_dir = root / "entries";
  boost::filesystem::create_directories(entries_dir);

  std::ostringstream desc_name;
  desc_name << "entries/" << std::setw(6) << std::setfill('0') << entry.index << ".btc";

  const boost::filesystem::path desc_path = root / desc_name.str();
  std::ofstream ofs(desc_path.string(), std::ios::binary);
  if (!ofs.is_open()) {
    return false;
  }

  if (!writePod(ofs, kMagic) || !writePod(ofs, kVersion)) {
    return false;
  }

  const int32_t index = static_cast<int32_t>(entry.index);
  if (!writePod(ofs, index)) {
    return false;
  }

  if (!writeVec3(ofs, entry.position)) {
    return false;
  }

  const double qx = entry.orientation.x();
  const double qy = entry.orientation.y();
  const double qz = entry.orientation.z();
  const double qw = entry.orientation.w();
  if (!writePod(ofs, qx) || !writePod(ofs, qy) || !writePod(ofs, qz) || !writePod(ofs, qw)) {
    return false;
  }

  if (!writeString(ofs, entry.pcd_path)) {
    return false;
  }
  if (!writeString(ofs, entry.plane_path)) {
    return false;
  }

  const uint32_t std_count = static_cast<uint32_t>(entry.stds.size());
  if (!writePod(ofs, std_count)) {
    return false;
  }

  for (const auto& std_item : entry.stds) {
    if (!writeSTD(ofs, std_item)) {
      return false;
    }
  }

  std::ofstream index_ofs((root / "index.txt").string(), std::ios::app);
  if (!index_ofs.is_open()) {
    return false;
  }
  index_ofs << desc_path.string() << "\n";

  return true;
}

bool loadDatabase(const std::string& root_dir,
                  std::vector<DatabaseEntry>* entries,
                  std::string* error_msg) {
  entries->clear();

  const boost::filesystem::path root(root_dir);
  std::ifstream ifs((root / "index.txt").string());
  if (!ifs.is_open()) {
    if (error_msg) {
      *error_msg = "failed to open index.txt";
    }
    return false;
  }

  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }

    boost::filesystem::path entry_file(line);
    if (!entry_file.is_absolute()) {
      entry_file = root / entry_file;
    }

    std::ifstream efs(entry_file.string(), std::ios::binary);
    if (!efs.is_open()) {
      continue;
    }

    uint32_t magic = 0;
    uint32_t version = 0;
    if (!readPod(efs, &magic) || !readPod(efs, &version)) {
      continue;
    }
    if (magic != kMagic || version != kVersion) {
      continue;
    }

    DatabaseEntry entry;
    int32_t index = -1;
    if (!readPod(efs, &index)) {
      continue;
    }
    entry.index = index;

    if (!readVec3(efs, &entry.position)) {
      continue;
    }

    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    if (!readPod(efs, &qx) || !readPod(efs, &qy) || !readPod(efs, &qz) || !readPod(efs, &qw)) {
      continue;
    }
    entry.orientation = Eigen::Quaterniond(qw, qx, qy, qz);
    entry.orientation.normalize();

    if (!readString(efs, &entry.pcd_path)) {
      continue;
    }
    if (!readString(efs, &entry.plane_path)) {
      continue;
    }

    uint32_t std_count = 0;
    if (!readPod(efs, &std_count)) {
      continue;
    }

    entry.stds.clear();
    entry.stds.reserve(std_count);
    bool valid_stds = true;
    for (uint32_t i = 0; i < std_count; ++i) {
      STD std_item;
      if (!readSTD(efs, &std_item)) {
        valid_stds = false;
        break;
      }
      entry.stds.push_back(std_item);
    }
    if (!valid_stds) {
      continue;
    }

    boost::filesystem::path plane_path(entry.plane_path);
    if (!plane_path.is_absolute()) {
      plane_path = root / plane_path;
    }

    entry.plane_cloud.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    if (pcl::io::loadPCDFile(plane_path.string(), *entry.plane_cloud) != 0 || entry.plane_cloud->empty()) {
      continue;
    }

    entry.desc_path = entry_file.string();
    entries->push_back(entry);
  }

  if (entries->empty()) {
    if (error_msg) {
      *error_msg = "database loaded but no valid entries";
    }
    return false;
  }

  std::sort(entries->begin(), entries->end(),
            [](const DatabaseEntry& a, const DatabaseEntry& b) { return a.index < b.index; });

  // STDescManager relies on frame_number being an array-like index.
  for (size_t i = 0; i < entries->size(); ++i) {
    (*entries)[i].index = static_cast<int>(i);
    for (auto& std_item : (*entries)[i].stds) {
      std_item.frame_number_ = static_cast<int>(i);
    }
  }

  return true;
}

}  // namespace btc_init_localizer
