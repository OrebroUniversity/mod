#include <gtest/gtest.h>
#include <ompl/mod/samplers/IntensityMapSampler.h>

#include <boost/math/distributions/chi_squared.hpp>
#include <map>

#include "test_helpers.hpp"

using namespace mod_test;

namespace {

// 6 rows x 5 cols, 1 m cells at x in [0, 4], y in [0, 5]; three invalid cells.
constexpr size_t kRows = 6, kCols = 5;
const std::vector<std::pair<size_t, size_t>> kInvalid = {{0, 0}, {2, 3}, {5, 4}};

double qOf(size_t r, size_t c) { return 0.05 + 0.9 * static_cast<double>((r * kCols + c) % 7) / 7.0; }

bool isInvalidCell(size_t r, size_t c) {
  for (const auto &rc : kInvalid)
    if (rc.first == r && rc.second == c) return true;
  return false;
}

bool checker(double x, double y) {
  const long r = std::lround(y), c = std::lround(x);
  if (r < 0 || c < 0) return true;
  return !isInvalidCell(static_cast<size_t>(r), static_cast<size_t>(c));
}

struct Fixture {
  ob::SpaceInformationPtr si;
  ob::ProblemDefinitionPtr pdef;
  ::MoD::IntensityMapConstPtr map;
  std::shared_ptr<ompl::MoD::IntensityMapSampler> sampler;

  explicit Fixture(double bias) {
    std::vector<double> values(kRows * kCols);
    for (size_t r = 0; r < kRows; ++r)
      for (size_t c = 0; c < kCols; ++c) values[r * kCols + c] = qOf(r, c);
    const std::string file = writeIntensityXml("q6x5.xml", 1.0, 0.0, 0.0, kRows, kCols, values);
    map = std::make_shared<const ::MoD::IntensityMap>(file);
    si = makeSE2(-0.5, 4.5, -0.5, 5.5, checker);
    pdef = makeProblem(si, {0.0, 1.0, 0.0}, {4.0, 4.0, 0.0});
    MoD::SamplerParameters p;
    p.type = MoD::SamplerType::intensity;
    p.bias = bias;
    sampler = std::make_shared<ompl::MoD::IntensityMapSampler>(pdef, 100, p, map);
  }

  /// Draws n samples and returns hit counts per (row, col) cell.
  std::map<std::pair<size_t, size_t>, size_t> draw(size_t n) {
    std::map<std::pair<size_t, size_t>, size_t> counts;
    ob::ScopedState<ob::SE2StateSpace> s(si);
    for (size_t i = 0; i < n; ++i) {
      sampler->sampleUniform(s.get(), ob::Cost(1e9));
      const long r = std::lround(s->getY()), c = std::lround(s->getX());
      EXPECT_TRUE(r >= 0 && c >= 0 && r < long(kRows) && c < long(kCols)) << "draw outside the map";
      ++counts[{size_t(r), size_t(c)}];
    }
    return counts;
  }
};

}  // namespace

TEST(IntensitySampler, ValidCellCount) {
  Fixture f(0.5);
  EXPECT_EQ(f.sampler->size(), kRows * kCols - kInvalid.size());
}

TEST(IntensitySampler, DrawsNeverHitInvalidCells) {
  Fixture f(0.5);
  const auto counts = f.draw(100000);
  for (const auto &rc : kInvalid) EXPECT_EQ(counts.count(rc), 0u) << "invalid cell hit";
  // Every valid cell is reachable.
  EXPECT_EQ(counts.size(), kRows * kCols - kInvalid.size());
}

TEST(IntensitySampler, UniformBranchIsUniformOverValidCells) {
  ompl::RNG::setSeed(11);
  Fixture f(0.0);
  const size_t n = 200000;
  const auto counts = f.draw(n);
  const size_t valid = kRows * kCols - kInvalid.size();
  ASSERT_EQ(counts.size(), valid) << "uniform branch cannot reach every valid cell";
  const double expected = static_cast<double>(n) / static_cast<double>(valid);
  double chi2 = 0.0;
  for (const auto &kv : counts) {
    const double diff = static_cast<double>(kv.second) - expected;
    chi2 += diff * diff / expected;
  }
  boost::math::chi_squared dist(static_cast<double>(valid - 1));
  const double p = boost::math::cdf(boost::math::complement(dist, chi2));
  EXPECT_GT(p, 0.01) << "chi2 = " << chi2;
}

TEST(IntensitySampler, QBranchFrequenciesFollowOneMinusQ) {
  // 5M draws: the per-cell relative sigma is ~0.23 %, so 1 % is more than 4 sigma. Seeded for determinism.
  ompl::RNG::setSeed(7);
  Fixture f(1.0);
  const size_t n = 5000000;
  const auto counts = f.draw(n);
  double sum = 0.0;
  for (size_t r = 0; r < kRows; ++r)
    for (size_t c = 0; c < kCols; ++c)
      if (!isInvalidCell(r, c)) sum += 1.0 - qOf(r, c);
  for (size_t r = 0; r < kRows; ++r) {
    for (size_t c = 0; c < kCols; ++c) {
      if (isInvalidCell(r, c)) continue;
      const double expected = (1.0 - qOf(r, c)) / sum;
      const auto it = counts.find({r, c});
      ASSERT_NE(it, counts.end()) << "cell (" << r << ", " << c << ") never drawn";
      const double freq = static_cast<double>(it->second) / static_cast<double>(n);
      EXPECT_NEAR(freq / expected, 1.0, 0.01) << "cell (" << r << ", " << c << ")";
    }
  }
}
