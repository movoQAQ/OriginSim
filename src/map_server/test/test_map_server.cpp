#include "map_server/costmap_2d.hpp"
#include "map_server/edt_2d.hpp"
#include "map_server/map_server.hpp"
#include "map_server/radar_rasterizer.hpp"
#include "topology_global_planner/grid/yastar_core.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <thread>
#include <unistd.h>
namespace
{
using namespace map_server;
using namespace std::chrono_literals;
struct Files
{
  std::filesystem::path dir;
  StaticMapFiles files;
  Files()
  {
    static int id = 0;
    dir = std::filesystem::temp_directory_path() /
          ("origin-map-test-" + std::to_string(getpid()) + "-" + std::to_string(id++));
    std::filesystem::create_directories(dir);
    cv::Mat occupancy(40, 40, CV_8UC1, cv::Scalar(255));
    occupancy.at<uint8_t>(20, 20) = 0;
    cv::imwrite((dir / "map.pgm").string(), occupancy);
    write("map.yaml",
          "image: map.pgm\nresolution: 0.1\norigin: [-2, -2, 0]\nnegate: 0\nfree_thresh: "
          "0.25\noccupied_thresh: 0.65\nmode: trinary\n");
    files.occupancy_yaml = (dir / "map.yaml").string();
    write("static.pcd",
          "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "
          "1\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 1\nDATA ascii\n-1 -1 0\n");
  }
  void write(const std::string& name, const std::string& text)
  {
    std::ofstream(dir / name) << text;
  }
  ~Files() { std::filesystem::remove_all(dir); }
};
BinaryGrid
grid(unsigned w = 30, unsigned h = 30, double r = 0.1)
{
  return { { w, h, r, 0, 0 }, std::vector<uint8_t>(w * h, 0) };
}
DerivedMaps
derive(BinaryGrid b, CostmapOptions o = {})
{
  DerivedMaps m;
  m.geometry = b.geometry;
  m.fused_obstacles = b;
  m.distance_field = preciseEdt(b);
  generateCosts(m, o);
  return m;
}
RadarTargetFrame
radar(RadarSource source, double stamp, double received, std::vector<RadarTarget> t = {})
{
  RadarTargetFrame f;
  f.source = source;
  f.observation_stamp_s = stamp;
  f.received_stamp_s = received;
  f.targets = std::move(t);
  return f;
}
RadarTarget
target(int id, double x, double y, double time = 10, double received = 100, double r = 0.2)
{
  return { static_cast<uint16_t>(id), { x, y }, time, received, r };
}
template<class F>
bool
until(F f)
{
  for (int i = 0; i < 200; ++i)
  {
    if (f())
      return true;
    std::this_thread::sleep_for(5ms);
  }
  return false;
}
TEST(Geometry, HalfOpenAndNonfinite)
{
  GridGeometry g{ 4, 3, 0.5, -1, -2 };
  g.validate();
  EXPECT_EQ(g.index({ -1, -2 }), 0u);
  EXPECT_FALSE(g.index({ 1, -2 }));
  EXPECT_FALSE(g.index({ -1, -0.5 }));
  EXPECT_FALSE(g.index({ NAN, 0 }));
  EXPECT_FALSE(g.index({ -1.01, -2 }));
  for (size_t i = 0; i < g.size(); ++i)
    EXPECT_EQ(g.index(g.center(i)), i);
}
TEST(Geometry, InvalidAndSize)
{
  EXPECT_THROW((GridGeometry{ 0, 2, 1, 0, 0 }.validate()), std::invalid_argument);
  EXPECT_THROW((GridGeometry{ 2, 2, NAN, 0, 0 }.validate()), std::invalid_argument);
  BinaryGrid b{ { 2, 2, 1, 0, 0 }, { 0 } };
  EXPECT_THROW(b.validate(), std::invalid_argument);
}
TEST(Static, MissingAndUnknown)
{
  Files f;
  EXPECT_EQ(loadStaticMapFiles(f.files).region_id.data[0], INVALID_REGION);
  cv::Mat image(40, 40, CV_8UC1, cv::Scalar(128));
  cv::imwrite((f.dir / "map.pgm").string(), image);
  EXPECT_THROW(loadStaticMapFiles(f.files), std::invalid_argument);
}
TEST(Static, RegionCanCoverObstacle)
{
  Files f;
  cv::Mat region(40, 40, CV_16UC1, cv::Scalar(300));
  cv::imwrite((f.dir / "regions.png").string(), region);
  f.write("region.yaml", "image: regions.png\nresolution: 0.1\norigin: [-2,-2,0]\nlabels: [300]\n");
  f.files.region_yaml = (f.dir / "region.yaml").string();
  auto m = loadStaticMapFiles(f.files);
  size_t i = 19 * 40 + 20;
  EXPECT_EQ(m.occupancy.data[i], 1);
  EXPECT_EQ(m.region_id.data[i], 300);
  MapServer server;
  ASSERT_TRUE(server.loadStaticMap(f.files));
  EXPECT_EQ(server.regionAt(m.geometry.center(i)), 300);
  EXPECT_EQ(server.regionAt({ 99, 99 }), INVALID_REGION);
}
TEST(Static, RejectYawGeometryAndLabels)
{
  Files f;
  f.write("bad.yaml", "image: map.pgm\nresolution: 0.1\norigin: [-2,-2,0.1]\n");
  f.files.occupancy_yaml = (f.dir / "bad.yaml").string();
  EXPECT_THROW(loadStaticMapFiles(f.files), std::invalid_argument);
}
TEST(Static, RejectRegionBitDepth)
{
  Files f;
  f.write("region.yaml", "image: map.pgm\nresolution: 0.1\norigin: [-2,-2,0]\nlabels: [1]\n");
  f.files.region_yaml = (f.dir / "region.yaml").string();
  EXPECT_THROW(loadStaticMapFiles(f.files), std::invalid_argument);
}
TEST(Static, HeightOnlyQueries)
{
  Files f;
  cv::Mat height(40, 40, CV_32FC1, cv::Scalar(123));
  cv::imwrite((f.dir / "height.tiff").string(), height);
  f.write("height.yaml", "image: height.tiff\nresolution: 0.1\norigin: [-2,-2,0]\n");
  f.files.height_yaml = (f.dir / "height.yaml").string();
  MapServer server;
  ASSERT_TRUE(server.loadStaticMap(f.files));
  EXPECT_EQ(server.heightAt({ 0, 0 }), 123);
  EXPECT_FALSE(server.depthAt({ 0, 0 }));
  EXPECT_FALSE(server.heightAt({ 9, 9 }));
}
TEST(Radar, IndependentSourcesAndSelf)
{
  RadarOptions o;
  o.self_id = 7;
  RadarTargetCache allies(o), enemies(o);
  ASSERT_TRUE(allies.update(radar(RadarSource::ALLY, 10, 100, { target(7, 1, 1), target(8, 2, 2) }),
                            RadarSource::ALLY));
  ASSERT_TRUE(
    enemies.update(radar(RadarSource::ENEMY, 10, 100, { target(7, 3, 3) }), RadarSource::ENEMY));
  EXPECT_EQ(allies.active(10.5, 100.5).size(), 1u);
  EXPECT_EQ(enemies.active(10.5, 100.5).size(), 1u);
  EXPECT_TRUE(allies.selfFilterVerified());
  allies.update(radar(RadarSource::ALLY, 11, 101, {}), RadarSource::ALLY);
  EXPECT_TRUE(allies.active(11, 101).empty());
  EXPECT_EQ(enemies.active(11, 101).size(), 1u);
}
TEST(Radar, OneHzNoFlickerThenStaleThenExpired)
{
  RadarTargetCache cache;
  cache.update(radar(RadarSource::ENEMY, 10, 100, { target(2, 1, 1) }), RadarSource::ENEMY);
  for (int i = 0; i < 20; ++i)
    EXPECT_EQ(cache.active(10 + i * 0.05, 100 + i * 0.05).size(), 1u);
  EXPECT_EQ(cache.status(12.6, 102.6), SourceStatus::STALE);
  EXPECT_EQ(cache.active(12.6, 102.6).size(), 1u);
  EXPECT_TRUE(cache.active(14.1, 104.1).empty());
  EXPECT_EQ(cache.status(14.1, 104.1), SourceStatus::STALE);
}
TEST(Radar, RejectLateInvalidAndMissingIdentity)
{
  RadarTargetCache cache;
  EXPECT_EQ(cache.status(10, 100), SourceStatus::MISSING);
  cache.update(radar(RadarSource::ALLY, 10, 100, { target(1, 1, 1) }), RadarSource::ALLY);
  EXPECT_FALSE(cache.update(radar(RadarSource::ALLY, 9, 101, {}), RadarSource::ALLY));
  EXPECT_EQ(cache.active(10, 100).size(), 1u);
  auto bad = radar(RadarSource::ALLY, 11, 101);
  bad.observation_valid = false;
  EXPECT_FALSE(cache.update(bad, RadarSource::ALLY));
  EXPECT_EQ(cache.status(11, 101), SourceStatus::ERROR);
  auto degraded =
    radar(RadarSource::ALLY, 12, 102, { target(0, 1, 1, 12, 102), target(0, 2, 2, 12, 102) });
  degraded.stable_ids = false;
  degraded.time_degraded = true;
  EXPECT_TRUE(cache.update(degraded, RadarSource::ALLY));
  EXPECT_FALSE(cache.selfFilterVerified());
  EXPECT_TRUE(cache.timeDegraded());
}
TEST(Radar, WallClockExpiryIsIndependent)
{
  RadarTargetCache cache;
  cache.update(radar(RadarSource::ALLY, 10, 100, { target(1, 1, 1) }), RadarSource::ALLY);
  EXPECT_EQ(cache.status(10, 103), SourceStatus::STALE);
  EXPECT_TRUE(cache.active(10, 105).empty());
}
TEST(Fusion, FarRadarAndStaticSurviveClear)
{
  auto b = grid(200, 50);
  b.data[1] = 1;
  rasterizeRadar(b, { target(4, 18, 2) });
  EXPECT_EQ(b.data[20 * 200 + 180], 1);
  EXPECT_EQ(b.data[1], 1);
}
TEST(Fusion, ConservativeOffsetProjection)
{
  auto b = grid(5, 5, 1);
  BinaryGrid local{ { 1, 1, 1, 0.5, 0.5 }, { 1 } };
  overlayConservative(b, local);
  EXPECT_EQ(b.data[0], 1);
  EXPECT_EQ(b.data[1], 1);
  EXPECT_EQ(b.data[5], 1);
  EXPECT_EQ(b.data[6], 1);
  EXPECT_EQ(b.data[2], 0);
}
TEST(EDT, AnalyticSingleSeed)
{
  auto b = grid(10, 10, 0.2);
  b.data[0] = 1;
  auto d = preciseEdt(b);
  EXPECT_FLOAT_EQ(d.data[0], 0);
  EXPECT_NEAR(d.data[4 * 10 + 3], 1.0, 1e-6);
}
TEST(EDT, NoSeedsAllSeedsAndNonbinary)
{
  auto b = grid(4, 4);
  auto d = preciseEdt(b);
  for (float v : d.data)
    EXPECT_TRUE(std::isinf(v));
  b.data.assign(16, 1);
  d = preciseEdt(b);
  for (float v : d.data)
    EXPECT_EQ(v, 0);
  b.data[3] = 2;
  EXPECT_THROW(preciseEdt(b), std::invalid_argument);
}
TEST(Cost, BoundaryAndPotential)
{
  CostmapOptions o;
  o.robot_radius = 0.1;
  o.safety_margin = 0.02;
  o.soft_radius = 0.8;
  auto b = grid(50, 50);
  b.data[25 * 50 + 25] = 1;
  auto m = derive(b, o);
  EXPECT_TRUE(std::isinf(m.costmap.data[0]));
  EXPECT_EQ(m.collision_mask.data[25 * 50 + 25], 1);
  EXPECT_GT(m.costmap.data[25 * 50 + 30], 1);
  EXPECT_FLOAT_EQ(m.costmap.data[25 * 50 + 40], 1);
  for (size_t i = 0; i < b.data.size(); ++i)
    EXPECT_EQ(m.collision_mask.data[i], std::isinf(m.costmap.data[i]));
}
TEST(Cost, RejectInvalidOptionsAndDistances)
{
  CostmapOptions o;
  o.beta = 0;
  EXPECT_THROW(o.validate(), std::invalid_argument);
  auto m = derive(grid());
  m.distance_field.data[0] = NAN;
  EXPECT_THROW(generateCosts(m, CostmapOptions{}), std::invalid_argument);
}
TEST(Cost, UnboundedMode)
{
  auto b = grid(100, 100);
  b.data[50 * 100 + 50] = 1;
  CostmapOptions o;
  o.tdt_unbounded_inverse_distance = true;
  auto m = derive(b, o);
  EXPECT_GT(m.costmap.data[50 * 100 + 90], 1);
}
TEST(Memory, HysteresisExpiryRollbackAndBinary)
{
  DynamicMemoryOptions o;
  o.hit_rate = 10;
  DynamicMemory memory(o);
  DynamicGrid g{ -1, -1, 4, 4, 0.1 };
  memory.advance(1, 100);
  memory.observe({ { 0, 0 } }, 1, 100);
  EXPECT_EQ(memory.snapshot(g).data[g.index({ 0, 0 })], 1);
  memory.advance(2.1, 101.1);
  EXPECT_EQ(memory.snapshot(g).data[g.index({ 0, 0 })], 0);
  EXPECT_FALSE(memory.advance(0.5, 102));
  EXPECT_EQ(memory.size(), 0u);
}
TEST(Cloud, StaticDifferenceAndFreshSupport)
{
  Files f;
  DynamicFilterOptions o;
  o.min_density = 1;
  o.min_component_area = 0.001;
  o.voxel_size = 0.005;
  o.static_voxel_size = 0.005;
  o.roi_min_radius = 0;
  DynamicCloudFilter filter(o, (f.dir / "static.pcd").string());
  DynamicCloud input;
  input.push_back({ -1, -1, 0 });
  auto fresh = filter.prepare(input, Eigen::Vector3d::Zero());
  DynamicGrid g{ -20, -20, 40, 40, 0.1 };
  auto result =
    filter.detect(*fresh, *fresh, Eigen::Vector3d::Zero(), g);
  EXPECT_TRUE(result.cells.empty());
  input.clear();
  for (int i = 0; i < 20; ++i)
    input.push_back({ 1.0f + i * 0.01f, 0.0f, 0.2f });
  fresh = filter.prepare(input, Eigen::Vector3d::Zero());
  result = filter.detect(*fresh, *fresh, Eigen::Vector3d::Zero(), g);
  EXPECT_FALSE(result.cells.empty());
  DynamicCloud empty;
  result = filter.detect(*fresh, empty, Eigen::Vector3d::Zero(), g);
  EXPECT_TRUE(result.cells.empty());
}
TEST(Worker, WholeVersionImmutabilityAndIndependentSources)
{
  Files f;
  MapServer server;
  ASSERT_TRUE(server.loadStaticMap(f.files));
  server.advanceTime(10, 100);
  server.start();
  ASSERT_TRUE(until([&] { return bool(server.getDerivedMaps()); }));
  auto old = server.getDerivedMaps();
  server.updateRadarAllies(radar(RadarSource::ALLY, 10, 100, { target(1, 1, 1) }));
  ASSERT_TRUE(until([&] { return server.getDerivedMaps()->version > old->version; }));
  auto next = server.getDerivedMaps();
  auto reference = derive(next->fused_obstacles);
  EXPECT_EQ(reference.collision_mask.data, next->collision_mask.data);
  EXPECT_EQ(reference.costmap.data, next->costmap.data);
  EXPECT_EQ(old->fused_obstacles.data.size(), 1600u);
  EXPECT_EQ(old->version, 1u);
  server.advanceTime(10.1, 100.1);
  EXPECT_EQ(server.getDynamicHealth().radar_allies, SourceStatus::VALID);
  EXPECT_EQ(server.getDynamicHealth().radar_enemies, SourceStatus::MISSING);
  server.stop();
}
TEST(Worker, HealthOnlyDoesNotRegenerateAndRollbackClearsRadar)
{
  Files f;
  MapServer server;
  ASSERT_TRUE(server.loadStaticMap(f.files));
  server.advanceTime(10, 100);
  server.start();
  ASSERT_TRUE(until([&] { return bool(server.getDerivedMaps()); }));
  auto old = server.getDerivedMaps();
  server.updateRadarAllies(radar(RadarSource::ALLY, 10, 100, {}));
  server.advanceTime(10.1, 100.1);
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(server.getDerivedMaps()->version, old->version);
  server.updateRadarEnemies(radar(RadarSource::ENEMY, 10, 100, { target(1, 1, 1) }));
  ASSERT_TRUE(until([&] { return server.getDerivedMaps()->version > old->version; }));
  server.advanceTime(1, 101);
  EXPECT_EQ(server.getDynamicHealth().radar_enemies, SourceStatus::MISSING);
  EXPECT_TRUE(server.getLatestDynamicObstacles()->enemies.empty());
  server.stop();
}
TEST(Worker, CloudAndRadarTimeoutIndependently)
{
  Files f;
  f.files.static_pcd = (f.dir / "static.pcd").string();
  MapServer server;
  ASSERT_TRUE(server.loadStaticMap(f.files));
  server.advanceTime(10, 100);
  server.start();
  auto empty = std::make_shared<DynamicCloud>();
  server.updateRegisteredCloud(
    { "map", 10, 100, empty, Eigen::Vector3d::Zero(), Eigen::Isometry3d::Identity() });
  ASSERT_TRUE(until([&] { return server.getDynamicHealth().cloud == SourceStatus::VALID; }));
  server.updateRadarEnemies(radar(RadarSource::ENEMY, 10, 100, { target(1, 1, 1) }));
  server.advanceTime(10.7, 100.7);
  EXPECT_EQ(server.getDynamicHealth().cloud, SourceStatus::STALE);
  EXPECT_EQ(server.getDynamicHealth().radar_enemies, SourceStatus::VALID);
  server.stop();
}
TEST(Worker, LocalizationJumpResetsCloudOnly)
{
  Files f;
  f.files.static_pcd = (f.dir / "static.pcd").string();
  MapServerConfig c;
  c.cloud_filter.roi_min_radius = 0;
  c.cloud_filter.voxel_size = 0.005;
  c.cloud_filter.min_density = 1;
  c.cloud_filter.min_component_area = 0.001;
  c.cloud_memory.hit_rate = 30;
  MapServer server(c);
  ASSERT_TRUE(server.loadStaticMap(f.files));
  server.advanceTime(10, 100);
  server.start();
  auto input = std::make_shared<DynamicCloud>();
  for (int i = 0; i < 20; ++i)
    input->push_back({ 1.0f + i * 0.01f, 0.0f, 0.2f });
  server.updateRadarEnemies(radar(RadarSource::ENEMY, 10, 100, { target(1, -1, 1) }));
  server.updateRegisteredCloud(
    { "map", 10, 100, input, Eigen::Vector3d::Zero(), Eigen::Isometry3d::Identity() });
  ASSERT_TRUE(until(
    [&]
    {
      auto v = server.getLatestDynamicObstacles();
      return v->cloud_mask &&
             std::count(v->cloud_mask->data.begin(), v->cloud_mask->data.end(), 1) > 0;
    }));
  server.advanceTime(10.1, 100.1);
  auto jump = Eigen::Isometry3d::Identity();
  jump.translation().x() = 1;
  server.updateRegisteredCloud(
    { "map", 10.1, 100.1, std::make_shared<DynamicCloud>(), Eigen::Vector3d::Zero(), jump });
  ASSERT_TRUE(until(
    [&]
    {
      auto v = server.getLatestDynamicObstacles();
      return v->cloud_mask &&
             std::count(v->cloud_mask->data.begin(), v->cloud_mask->data.end(), 1) == 0;
    }));
  EXPECT_EQ(server.getLatestDynamicObstacles()->enemies.size(), 1u);
  server.stop();
}
TEST(Worker, StopAndRestart)
{
  Files f;
  MapServer server;
  ASSERT_TRUE(server.loadStaticMap(f.files));
  server.advanceTime(1, 100);
  server.start();
  server.start();
  server.stop();
  server.stop();
  server.start();
  server.stop();
}
TEST(Search, InfinityAndCornerCutting)
{
  using topology_global_planner::grid::YAstarCore;
  DerivedMaps m;
  m.geometry = { 3, 3, 1, 0, 0 };
  m.costmap = { m.geometry, std::vector<float>(9, 1) };
  m.collision_mask = { m.geometry, std::vector<uint8_t>(9, 0) };
  for (size_t i : { 1u, 3u })
  {
    m.costmap.data[i] = INFINITY;
    m.collision_mask.data[i] = 1;
  }
  EXPECT_TRUE(YAstarCore::search(m, 0, 8, 0, [] { return false; }).empty());
  m.costmap.data[1] = 1;
  m.collision_mask.data[1] = 0;
  auto path = YAstarCore::search(m, 0, 8, 0, [] { return false; });
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(path[1], 1u);
  m.costmap.data[0] = INFINITY;
  m.collision_mask.data[0] = 1;
  EXPECT_THROW(YAstarCore::search(m, 0, 8, 0, [] { return false; }), std::runtime_error);
}
TEST(Search, ContinuousCostAvoidsExpensiveBand)
{
  using topology_global_planner::grid::YAstarCore;
  DerivedMaps m;
  m.geometry = { 5, 5, 1, 0, 0 };
  m.costmap = { m.geometry, std::vector<float>(25, 1) };
  m.collision_mask = { m.geometry, std::vector<uint8_t>(25, 0) };
  for (size_t i : { 11u, 12u, 13u })
    m.costmap.data[i] = 100;
  auto path = YAstarCore::search(m, 10, 14, 0, [] { return false; });
  for (auto i : path)
    EXPECT_LT(m.costmap.data[i], 100);
}
TEST(View, RetainsSameDerivedPointer)
{
  auto m = std::make_shared<DerivedMaps>(derive(grid()));
  Costmap2D a;
  a.setSnapshot(m);
  Costmap2D b = a;
  EXPECT_EQ(a.getDerivedMaps(), b.getDerivedMaps());
  EXPECT_EQ(b.getDerivedMaps()->collision_mask.data, m->collision_mask.data);
}
TEST(Cost,RejectFloatOverflowInsteadOfFalseCollision) {
  auto b=grid(100,100);b.data[50*100+50]=1;
  CostmapOptions options;options.alpha=1e40;
  EXPECT_THROW(derive(b,options),std::overflow_error);
}
TEST(Worker,NonfiniteCloudIsAnErrorNotAnEmptyObservation) {
  Files f;f.files.static_pcd=(f.dir/"static.pcd").string();MapServer server;
  ASSERT_TRUE(server.loadStaticMap(f.files));server.advanceTime(10,100);server.start();
  auto input=std::make_shared<DynamicCloud>();input->push_back({NAN,NAN,NAN});
  server.updateRegisteredCloud({"map",10,100,input,Eigen::Vector3d::Zero(),Eigen::Isometry3d::Identity()});
  ASSERT_TRUE(until([&]{return server.getDynamicHealth().cloud==SourceStatus::ERROR;}));
  server.stop();
}

}
