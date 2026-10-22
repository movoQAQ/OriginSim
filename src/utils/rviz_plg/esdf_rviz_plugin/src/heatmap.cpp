// Uses the palette extension points of ROS 2 RViz MapDisplay (BSD-3-Clause).
#include <algorithm>
#include <atomic>
#include <array>
#include <vector>
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
  // Restore the first ESDF view's light-blue / green / yellow / red band.
  const std::array<std::array<float,3>,4> colors{{
    {{1,0,0}}, {{1,1,0}}, {{0,1,0}}, {{0.55f,0.85f,1}}
  }};
  for(int i=0;i<256;++i){
   const float h=3*std::clamp(i/100.0f,0.0f,1.0f);
   const int a=std::min(static_cast<int>(h),2);
   const float fraction=h-a;
   for(int channel=0;channel<3;++channel)
    bytes[i*4+channel]=255*((1-fraction)*colors[a][channel]+fraction*colors[a+1][channel]);
  }
  bytes[255*4+3]=0;
  static std::atomic<unsigned> counter{0};
  Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(bytes.data(),bytes.size()));
  auto texture=Ogre::TextureManager::getSingleton().loadRawData(
    "EsdfCostPalette"+std::to_string(counter++),"rviz_rendering",stream,256,1,Ogre::PF_BYTE_RGBA,Ogre::TEX_TYPE_1D,0);
  palette_textures_[2]=texture;palette_textures_binary_[2]=texture;
  color_scheme_property_->setString("raw");color_scheme_property_->hide();
  binary_view_property_->setBool(false);binary_view_property_->hide();
 }
};
}
PLUGINLIB_EXPORT_CLASS(esdf_rviz_plugin::Heatmap,rviz_common::Display)
