#include "map_server/costmap_generator.hpp"
#include "map_server/edt_2d.hpp"
#include "map_server/dynamic_cloud_filter.hpp"
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <unistd.h>
#include "map_server/radar_rasterizer.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <random>
#include <sys/resource.h>
int
main(int argc, char** argv)
{
  const int width = argc > 1 ? std::stoi(argv[1]) : 560,
            height = argc > 2 ? std::stoi(argv[2]) : 300,
            iterations = argc > 3 ? std::stoi(argv[3]) : 100;
  if (width <= 0 || height <= 0 || iterations < 3)
    return 1;
  map_server::GridGeometry g{
    static_cast<unsigned>(width), static_cast<unsigned>(height), 0.05, 0, 0
  };
  g.validate();
  map_server::BinaryGrid static_map{ g, std::vector<uint8_t>(g.size(), 0) };
  std::mt19937 generator(42);
  for (size_t i = 0; i < g.size(); ++i)
    if (generator() % 1000 == 0)
      static_map.data[i] = 1;
  using Clock = std::chrono::steady_clock;
  std::vector<double> fusion, edt, cost, total;
  const auto ms = [](auto a, auto b)
  { return std::chrono::duration<double, std::milli>(b - a).count(); };
  for (int i = 0; i < iterations + 5; ++i)
  {
    auto begin = Clock::now();
    map_server::DerivedMaps m;
    m.geometry = g;
    m.fused_obstacles = static_map;
    map_server::RadarTarget target{
      1, { g.width * g.resolution * 0.7, g.height * g.resolution * 0.6 }, 0, 0, 0.3 + 0.001 * i
    };
    map_server::rasterizeObstacles(m.fused_obstacles, { {target.xy_map, target.physical_radius_m} });
    auto f = Clock::now();
    m.distance_field = map_server::preciseEdt(m.fused_obstacles);
    auto e = Clock::now();
    map_server::generateCosts(m, {});
    auto c = Clock::now();
    if (i >= 5)
    {
      fusion.push_back(ms(begin, f));
      edt.push_back(ms(f, e));
      cost.push_back(ms(e, c));
      total.push_back(ms(begin, c));
    }
  }
  std::cout << "width,height,resolution,iterations\n"
            << width << "," << height << ",0.05," << iterations << "\nstage,p50_ms,p95_ms,max_ms\n";
  const auto report = [](const char* name, std::vector<double> values)
  {
    std::sort(values.begin(), values.end());
    std::cout << name << "," << values[values.size() / 2] << ","
              << values[static_cast<size_t>((values.size() - 1) * 0.95)] << "," << values.back()
              << "\n";
  };
  report("fusion", fusion);
  report("edt", edt);
  report("costmap", cost);
  report("derived_total", total);
  // Separate synthetic cloud benchmark, not a real sensor/PCD alignment test.
  const auto pcd = std::filesystem::temp_directory_path() /
    ("origin-cloud-benchmark-"+std::to_string(getpid())+".pcd");
  map_server::DynamicCloud reference, frame;
  for(int y=0;y<100;++y)for(int x=0;x<100;++x)
    reference.push_back({-4.0f+x*0.08f,-4.0f+y*0.08f,0.0f});
  frame=reference;
  for(int i=0;i<1000;++i)frame.push_back({1.0f+(i%50)*0.005f,1.0f+(i/50)*0.005f,0.3f});
  pcl::io::savePCDFileBinary(pcd.string(),reference);
  map_server::DynamicFilterOptions options;
  map_server::DynamicCloudFilter filter(options,pcd.string());
  std::filesystem::remove(pcd);
  map_server::DynamicGrid local{-160,-160,320,320,0.05};
  std::vector<double> prepare,difference;
  for(int i=0;i<iterations+5;++i){
    auto begin=Clock::now();auto fresh=filter.prepare(frame,Eigen::Vector3d::Zero());
    auto cropped=Clock::now();auto result=filter.detect(*fresh,*fresh,Eigen::Vector3d::Zero(),local);
    auto finished=Clock::now();
    if(i>=5){prepare.push_back(ms(begin,cropped));difference.push_back(ms(cropped,finished));}
  }
  report("cloud_prepare_11000_points",prepare);
  report("pcd_difference_filters_10000_reference_points",difference);
  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
  std::cout << "peak_rss_kib," << usage.ru_maxrss << "\n";
}
