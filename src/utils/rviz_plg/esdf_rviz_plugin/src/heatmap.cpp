// Uses the palette extension points of ROS 2 RViz MapDisplay (BSD-3-Clause).
#include <atomic>
#include <cmath>
#include <vector>
#include <rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp>
#include <OgreTextureManager.h>
#include <OgreDataStream.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_default_plugins/displays/map/map_display.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
namespace esdf_rviz_plugin {
class Heatmap : public rviz_default_plugins::displays::MapDisplay {
public:
 void onInitialize() override {
  MapDisplay::onInitialize();
  std::vector<unsigned char> bytes(256*4,255);
  // Same native Rainbow function and non-inverted Intensity direction as
  // bit-nav's 2DESDF: near red, then yellow/green/cyan/blue, far violet.
  for(int i=0;i<=100;++i){
   Ogre::ColourValue color;
   rviz_default_plugins::getRainbowColor(1.0f-i/100.0f,color);
   bytes[i*4]=std::lround(255*color.r);
   bytes[i*4+1]=std::lround(255*color.g);
   bytes[i*4+2]=std::lround(255*color.b);
  }
  bytes[255*4+3]=0;
  static std::atomic<unsigned> counter{0};
  Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(bytes.data(),bytes.size()));
  auto texture=Ogre::TextureManager::getSingleton().loadRawData(
    "EsdfDistancePalette"+std::to_string(counter++),"rviz_rendering",stream,256,1,Ogre::PF_BYTE_RGBA,Ogre::TEX_TYPE_1D,0);
  palette_textures_[2]=texture;palette_textures_binary_[2]=texture;
  color_scheme_property_->setString("raw");color_scheme_property_->hide();
  binary_view_property_->setBool(false);binary_view_property_->hide();
 }
};
}
PLUGINLIB_EXPORT_CLASS(esdf_rviz_plugin::Heatmap,rviz_common::Display)
