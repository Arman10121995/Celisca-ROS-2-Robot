#include <gtest/gtest.h>

#include <vector>

#include "robot_lab_planning/dijkstra_planner.hpp"

using robot_lab_planning::DijkstraPlanner;

namespace
{
constexpr unsigned char kFree = 0;
constexpr unsigned char kLethal = 254;
constexpr unsigned char kUnknown = 255;

std::vector<unsigned int> plan(
  const std::vector<unsigned char> & grid, unsigned int width,
  unsigned int start, unsigned int goal, bool allow_unknown = true)
{
  return DijkstraPlanner::search(
    grid.data(), width, grid.size() / width, start, goal, allow_unknown, 253, 3.0);
}
}  // namespace

TEST(DijkstraSearch, StraightLineOnFreeGrid)
{
  std::vector<unsigned char> grid(5 * 5, kFree);
  const auto path = plan(grid, 5, 2 * 5 + 0, 2 * 5 + 4);
  ASSERT_EQ(path.size(), 5u);
  for (unsigned int i = 0; i < path.size(); ++i) {
    EXPECT_EQ(path[i], 2 * 5 + i);
  }
}

TEST(DijkstraSearch, GoesAroundAWallThroughTheGap)
{
  // A wall in column 3 with a gap at row 4.
  const unsigned int w = 7;
  std::vector<unsigned char> grid(w * 7, kFree);
  for (unsigned int y = 0; y < 7; ++y) {
    if (y != 4) {
      grid[y * w + 3] = kLethal;
    }
  }
  const auto path = plan(grid, w, 1 * w + 1, 1 * w + 5);
  ASSERT_FALSE(path.empty());
  bool through_gap = false;
  for (const auto index : path) {
    EXPECT_NE(grid[index], kLethal);
    through_gap |= index == 4 * w + 3;
  }
  EXPECT_TRUE(through_gap);
}

TEST(DijkstraSearch, PrefersLowCostCellsOverTheShortestLine)
{
  // Row 1 is high cost; the path should leave it for row 0 or 2.
  const unsigned int w = 9;
  std::vector<unsigned char> grid(w * 3, kFree);
  for (unsigned int x = 1; x < w - 1; ++x) {
    grid[1 * w + x] = 200;
  }
  const auto path = plan(grid, w, 1 * w + 0, 1 * w + w - 1);
  ASSERT_FALSE(path.empty());
  int on_costly_row = 0;
  for (const auto index : path) {
    on_costly_row += grid[index] == 200;
  }
  EXPECT_EQ(on_costly_row, 0);
}

TEST(DijkstraSearch, NoDiagonalCutBetweenBlockedCells)
{
  // Two blocked cells touching at a corner; the diagonal between them is closed.
  std::vector<unsigned char> grid = {
    kFree, kLethal,
    kLethal, kFree,
  };
  EXPECT_TRUE(plan(grid, 2, 0, 3).empty());
}

TEST(DijkstraSearch, BlockedGoalAndUnreachableGoalFail)
{
  std::vector<unsigned char> grid(3 * 3, kFree);
  grid[8] = kLethal;
  EXPECT_TRUE(plan(grid, 3, 0, 8).empty());
  std::vector<unsigned char> walled = {
    kFree, kLethal, kFree,
    kFree, kLethal, kFree,
    kFree, kLethal, kFree,
  };
  EXPECT_TRUE(plan(walled, 3, 0, 2).empty());
}

TEST(DijkstraSearch, UnknownCellsFollowAllowUnknown)
{
  std::vector<unsigned char> grid = {kFree, kUnknown, kFree};
  EXPECT_EQ(plan(grid, 3, 0, 2, true).size(), 3u);
  EXPECT_TRUE(plan(grid, 3, 0, 2, false).empty());
}

TEST(DijkstraSearch, StartInsideTheInflationBandStillPlans)
{
  std::vector<unsigned char> grid = {253, kFree, kFree};
  EXPECT_EQ(plan(grid, 3, 0, 2).size(), 3u);
}

TEST(DijkstraSearch, StartEqualsGoal)
{
  std::vector<unsigned char> grid(4, kFree);
  const auto path = plan(grid, 2, 3, 3);
  ASSERT_EQ(path.size(), 1u);
  EXPECT_EQ(path[0], 3u);
}
