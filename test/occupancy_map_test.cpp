#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "core/occupancy_map.hpp"

using MoD::playground::OccupancyMap;

namespace {
const std::string kData = MOD_TEST_DATA_DIR;
}

TEST(OccupancyMap, ATC) {
  OccupancyMap map(kData + "/atc/atc.yaml");
  EXPECT_EQ(map.width(), 2800u);
  EXPECT_EQ(map.height(), 1200u);
  EXPECT_DOUBLE_EQ(map.pixel_size(), 0.05);
  const auto b = map.bounds();
  EXPECT_DOUBLE_EQ(b.x_min, -60.0);
  EXPECT_DOUBLE_EQ(b.x_max, 80.0);
  EXPECT_DOUBLE_EQ(b.y_min, -40.0);
  EXPECT_DOUBLE_EQ(b.y_max, 20.0);
  EXPECT_TRUE(map.occupied(-59.0, -39.0));  // black corner
  EXPECT_FALSE(map.occupied(0.0, 0.0));
  EXPECT_FALSE(map.occupied(-0.133, -8.555));  // scenario 3 start
  EXPECT_FALSE(map.occupied(47.69, -18.848));  // scenario 1 start
  EXPECT_TRUE(map.occupied(-1000.0, 0.0));     // outside
  EXPECT_TRUE(map.occupied(80.0, 0.0));        // just outside the right edge
  EXPECT_EQ(map.gray().size(), 2800u * 1200u);
  EXPECT_GT(map.occupiedCount(), 0u);
  EXPECT_LT(map.occupiedCount(), map.width() * map.height());
}

TEST(OccupancyMap, YamlAndPgmParsing) {
  // 4 x 3 image with a comment line; row 0 of the file is the TOP of the image.
  const std::string dir = ::testing::TempDir();
  {
    std::ofstream pgm(dir + "/tiny.pgm", std::ios::binary);
    pgm << "P5\n# a comment\n4 3\n255\n";
    const unsigned char rows[3][4] = {{255, 255, 255, 0},     // top row: last pixel occupied
                                      {255, 100, 255, 255},   // 100 -> p = 0.61 < 0.65: free
                                      {0, 255, 255, 255}};    // bottom row: first pixel occupied
    for (const auto &r : rows) pgm.write(reinterpret_cast<const char *>(r), 4);
  }
  {
    std::ofstream yaml(dir + "/tiny.yaml");
    yaml << "image: tiny.pgm\nresolution: 0.5\norigin: [1.0, 2.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\n"
            "free_thresh: 0.196\n";
  }
  OccupancyMap map(dir + "/tiny.yaml");
  EXPECT_EQ(map.width(), 4u);
  EXPECT_EQ(map.height(), 3u);
  EXPECT_DOUBLE_EQ(map.pixel_size(), 0.5);
  EXPECT_DOUBLE_EQ(map.originX(), 1.0);
  EXPECT_DOUBLE_EQ(map.originY(), 2.0);
  EXPECT_TRUE(std::filesystem::equivalent(map.imagePath(), dir + "/tiny.pgm"));
  // Bottom-left pixel (col 0, row 0) is occupied: world (1.0..1.5, 2.0..2.5).
  EXPECT_TRUE(map.occupied(1.25, 2.25));
  EXPECT_TRUE(map.occupiedPixel(0, 0));
  // Top-right pixel (col 3, row 2) is occupied: world (2.5..3.0, 3.0..3.5).
  EXPECT_TRUE(map.occupied(2.75, 3.25));
  // Gray 100 pixel is free (unknown counts as free).
  EXPECT_FALSE(map.occupied(1.75, 2.75));
  EXPECT_FALSE(map.occupied(2.25, 2.25));
  EXPECT_EQ(map.occupiedCount(), 2u);
}

TEST(OccupancyMap, InMemory) {
  std::vector<uint8_t> occ(10 * 10, 0);
  occ[5 * 10 + 5] = 1;  // row 5, col 5
  OccupancyMap map(10, 10, 0.1, 0.0, 0.0, occ);
  EXPECT_TRUE(map.occupied(0.55, 0.55));
  EXPECT_FALSE(map.occupied(0.15, 0.15));
  EXPECT_TRUE(map.occupied(-0.01, 0.5));
  EXPECT_DOUBLE_EQ(map.bounds().x_max, 1.0);
}
