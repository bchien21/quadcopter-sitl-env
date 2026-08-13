#include <octomap/OcTree.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
constexpr std::uint8_t kFree = 0;
constexpr std::uint8_t kOccupied = 1;
constexpr std::uint8_t kUnknown = 2;

struct Bounds
{
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  double min_z;
  double max_z;
};

void print_usage(const char * program)
{
  std::cerr
    << "Usage: " << program
    << " INPUT.bt OUTPUT_PREFIX MIN_X MAX_X MIN_Y MAX_Y MIN_Z MAX_Z [FRAME_ID]\n";
}

std::size_t cell_count(double minimum, double maximum, double resolution)
{
  return static_cast<std::size_t>(std::ceil((maximum - minimum) / resolution));
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 9 && argc != 10) {
    print_usage(argv[0]);
    return 1;
  }

  try {
    const std::string input_path = argv[1];
    const std::string output_prefix = argv[2];
    const Bounds bounds{
      std::stod(argv[3]), std::stod(argv[4]),
      std::stod(argv[5]), std::stod(argv[6]),
      std::stod(argv[7]), std::stod(argv[8])};
    const std::string frame_id = argc == 10 ? argv[9] : "world";

    if (!(bounds.min_x < bounds.max_x && bounds.min_y < bounds.max_y &&
      bounds.min_z < bounds.max_z))
    {
      throw std::runtime_error("Each minimum bound must be less than its maximum bound.");
    }

    auto tree = std::make_unique<octomap::OcTree>(input_path);

    const double resolution = tree->getResolution();
    const std::size_t nx = cell_count(bounds.min_x, bounds.max_x, resolution);
    const std::size_t ny = cell_count(bounds.min_y, bounds.max_y, resolution);
    const std::size_t nz = cell_count(bounds.min_z, bounds.max_z, resolution);
    const double max_x = bounds.min_x + nx * resolution;
    const double max_y = bounds.min_y + ny * resolution;
    const double max_z = bounds.min_z + nz * resolution;

    if (nx == 0 || ny == 0 || nz == 0 || nx > SIZE_MAX / ny || nx * ny > SIZE_MAX / nz) {
      throw std::runtime_error("Requested grid dimensions are invalid or too large.");
    }

    std::vector<std::uint8_t> grid(nx * ny * nz, kUnknown);

    const auto first_cell = [resolution](double leaf_min, double grid_min) {
        return static_cast<long>(std::floor((leaf_min - grid_min) / resolution + 1e-9));
      };
    const auto past_last_cell = [resolution](double leaf_max, double grid_min) {
        return static_cast<long>(std::ceil((leaf_max - grid_min) / resolution - 1e-9));
      };

    for (auto leaf = tree->begin_leafs(), end = tree->end_leafs(); leaf != end; ++leaf) {
      const double half_size = leaf.getSize() / 2.0;
      const double leaf_min_x = leaf.getX() - half_size;
      const double leaf_max_x = leaf.getX() + half_size;
      const double leaf_min_y = leaf.getY() - half_size;
      const double leaf_max_y = leaf.getY() + half_size;
      const double leaf_min_z = leaf.getZ() - half_size;
      const double leaf_max_z = leaf.getZ() + half_size;

      if (leaf_max_x <= bounds.min_x || leaf_min_x >= max_x ||
        leaf_max_y <= bounds.min_y || leaf_min_y >= max_y ||
        leaf_max_z <= bounds.min_z || leaf_min_z >= max_z)
      {
        continue;
      }

      const long ix_begin = std::max(0L, first_cell(leaf_min_x, bounds.min_x));
      const long iy_begin = std::max(0L, first_cell(leaf_min_y, bounds.min_y));
      const long iz_begin = std::max(0L, first_cell(leaf_min_z, bounds.min_z));
      const long ix_end = std::min(static_cast<long>(nx), past_last_cell(leaf_max_x, bounds.min_x));
      const long iy_end = std::min(static_cast<long>(ny), past_last_cell(leaf_max_y, bounds.min_y));
      const long iz_end = std::min(static_cast<long>(nz), past_last_cell(leaf_max_z, bounds.min_z));
      const std::uint8_t value = tree->isNodeOccupied(*leaf) ? kOccupied : kFree;

      for (long ix = ix_begin; ix < ix_end; ++ix) {
        for (long iy = iy_begin; iy < iy_end; ++iy) {
          for (long iz = iz_begin; iz < iz_end; ++iz) {
            const std::size_t index =
              (static_cast<std::size_t>(ix) * ny + static_cast<std::size_t>(iy)) * nz +
              static_cast<std::size_t>(iz);
            if (value == kOccupied || grid[index] == kUnknown) {
              grid[index] = value;
            }
          }
        }
      }
    }

    const std::filesystem::path prefix(output_prefix);
    if (!prefix.parent_path().empty()) {
      std::filesystem::create_directories(prefix.parent_path());
    }

    std::ofstream binary(output_prefix + ".bin", std::ios::binary);
    binary.write(reinterpret_cast<const char *>(grid.data()), grid.size());
    if (!binary) {
      throw std::runtime_error("Could not write output grid.");
    }

    const auto free_count = std::count(grid.begin(), grid.end(), kFree);
    const auto occupied_count = std::count(grid.begin(), grid.end(), kOccupied);
    const auto unknown_count = std::count(grid.begin(), grid.end(), kUnknown);

    std::ofstream metadata(output_prefix + ".json");
    metadata
      << "{\n"
      << "  \"shape\": [" << nx << ", " << ny << ", " << nz << "],\n"
      << "  \"bounds_min\": [" << bounds.min_x << ", " << bounds.min_y << ", "
      << bounds.min_z << "],\n"
      << "  \"bounds_max\": [" << max_x << ", " << max_y << ", " << max_z << "],\n"
      << "  \"center\": [" << (bounds.min_x + max_x) / 2.0 << ", "
      << (bounds.min_y + max_y) / 2.0 << ", " << (bounds.min_z + max_z) / 2.0 << "],\n"
      << "  \"dimensions\": [" << max_x - bounds.min_x << ", "
      << max_y - bounds.min_y << ", " << max_z - bounds.min_z << "],\n"
      << "  \"voxel_size\": " << resolution << ",\n"
      << "  \"frame_id\": \"" << frame_id << "\",\n"
      << "  \"dtype\": \"uint8\",\n"
      << "  \"order\": \"C\",\n"
      << "  \"values\": {\"free\": 0, \"occupied\": 1, \"unknown\": 2}\n"
      << "}\n";
    if (!metadata) {
      throw std::runtime_error("Could not write grid metadata.");
    }

    std::cout << "Wrote " << nx << " x " << ny << " x " << nz << " grid at "
              << resolution << " m resolution\n"
              << "Free: " << free_count << ", occupied: " << occupied_count
              << ", unknown: " << unknown_count << '\n';
  } catch (const std::exception & error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }

  return 0;
}
