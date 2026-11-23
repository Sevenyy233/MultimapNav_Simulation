#include "clearing_robot/local_map/compound_display.hpp"

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <algorithm>
#include <cmath>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <limits>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/string_property.hpp>

namespace clearing_robot
{
namespace
{
/** @brief 蓝绿黄渐变，用固定上下限保证跨帧颜色含义一致。 */
Ogre::ColourValue ramp(float value, float lo, float hi, float alpha, int palette)
{
  const float t = hi > lo ? std::clamp((value - lo) / (hi - lo), 0.0f, 1.0f) : 0.5f;
  if (palette == 1)
    return Ogre::ColourValue(t, t, t, alpha);
  if (t <= 0.5f)
  {
    const float weight = 2.0f * t;
    return Ogre::ColourValue(0.15f + 0.05f * weight, 0.35f + 0.35f * weight, 0.80f - 0.44f * weight,
                             alpha);
  }
  const float weight = 2.0f * t - 1.0f;
  return Ogre::ColourValue(0.20f + 0.78f * weight, 0.70f + 0.12f * weight, 0.36f - 0.16f * weight,
                           alpha);
}
void vertex(Ogre::ManualObject *obj, float x, float y, float z, const Ogre::ColourValue &c)
{
  obj->position(x, y, z);
  obj->colour(c);
}
void quad(Ogre::ManualObject *o, const Ogre::Vector3 &a, const Ogre::Vector3 &b,
          const Ogre::Vector3 &c, const Ogre::Vector3 &d, const Ogre::ColourValue &color)
{
  for (const auto &p : {a, b, c, a, c, d})
    vertex(o, p.x, p.y, p.z, color);
}
void box(Ogre::ManualObject *o, float x0, float x1, float y0, float y1, float z0, float z1,
         const Ogre::ColourValue &c)
{
  quad(o, {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}, c);
  quad(o, {x0, y0, z0}, {x0, y1, z0}, {x1, y1, z0}, {x1, y0, z0}, c);
  quad(o, {x0, y0, z0}, {x1, y0, z0}, {x1, y0, z1}, {x0, y0, z1}, c);
  quad(o, {x1, y1, z0}, {x0, y1, z0}, {x0, y1, z1}, {x1, y1, z1}, c);
  quad(o, {x0, y1, z0}, {x0, y0, z0}, {x0, y0, z1}, {x0, y1, z1}, c);
  quad(o, {x1, y0, z0}, {x1, y1, z0}, {x1, y1, z1}, {x1, y0, z1}, c);
}
}  // namespace
CompoundDisplay::CompoundDisplay()
{
  using namespace rviz_common::properties;
  show_terrain_ = new BoolProperty("Show Terrain", true, "显示地形网格", this, SLOT(redraw()));
  show_occupied_ = new BoolProperty("Show Occupancy", true, "显示占据体素", this, SLOT(redraw()));
  height_layer_ =
      new StringProperty("Height Layer", "elevation", "地形高度图层", this, SLOT(redraw()));
  color_layer_ =
      new StringProperty("Color Layer", "elevation", "标量着色图层", this, SLOT(redraw()));
  terrain_mode_ = new EnumProperty("Terrain Color Mode", "Color Map", "固定颜色或标量渐变", this,
                                   SLOT(redraw()));
  terrain_mode_->addOption("Color Map", 0);
  terrain_mode_->addOption("Flat Color", 1);
  color_map_ =
      new EnumProperty("Color Map", "Blue-Green-Yellow", "标量及高度色带", this, SLOT(redraw()));
  color_map_->addOption("Blue-Green-Yellow", 0);
  color_map_->addOption("Grayscale", 1);
  terrain_color_ = new ColorProperty("Terrain Color", QColor(70, 170, 105), "固定地形颜色", this,
                                     SLOT(redraw()));
  scalar_min_ = new FloatProperty("Color Minimum", -1.0, "色带下限", this, SLOT(redraw()));
  scalar_max_ = new FloatProperty("Color Maximum", 1.0, "色带上限", this, SLOT(redraw()));
  terrain_alpha_ = new FloatProperty("Terrain Alpha", 0.55, "地形透明度", this, SLOT(redraw()));
  terrain_alpha_->setMin(0);
  terrain_alpha_->setMax(1);
  voxel_mode_ = new EnumProperty("Voxel Color Mode", "Flat Color", "固定颜色或按体素高度着色", this,
                                 SLOT(redraw()));
  voxel_mode_->addOption("Flat Color", 0);
  voxel_mode_->addOption("Height", 1);
  voxel_color_ =
      new ColorProperty("Voxel Color", QColor(245, 110, 35), "固定体素颜色", this, SLOT(redraw()));
  voxel_min_ =
      new FloatProperty("Voxel Height Minimum", -1.0, "高度色带下限", this, SLOT(redraw()));
  voxel_max_ = new FloatProperty("Voxel Height Maximum", 2.0, "高度色带上限", this, SLOT(redraw()));
  voxel_alpha_ = new FloatProperty("Voxel Alpha", 0.8, "体素透明度", this, SLOT(redraw()));
  voxel_alpha_->setMin(0);
  voxel_alpha_->setMax(1);
  depth_ = new IntProperty("Octree Depth", 16, "显示深度（1–16；较浅深度聚合体素）", this,
                           SLOT(redraw()));
  depth_->setMin(1);
  depth_->setMax(16);
}
CompoundDisplay::~CompoundDisplay()
{
  if (terrain_)
    scene_manager_->destroyManualObject(terrain_);
  if (occupied_)
    scene_manager_->destroyManualObject(occupied_);
  if (!material_name_.empty())
    Ogre::MaterialManager::getSingleton().remove(material_name_);
}
void CompoundDisplay::onInitialize()
{
  MFDClass::onInitialize();
  terrain_ = scene_manager_->createManualObject();
  occupied_ = scene_manager_->createManualObject();
  terrain_->setDynamic(true);
  occupied_->setDynamic(true);
  scene_node_->attachObject(terrain_);
  scene_node_->attachObject(occupied_);
  material_name_ = terrain_->getName() + "_compound_material";
  auto material = Ogre::MaterialManager::getSingleton().create(
      material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  auto *pass = material->getTechnique(0)->getPass(0);
  pass->setLightingEnabled(false);
  pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  pass->setDepthWriteEnabled(false);
  pass->setCullingMode(Ogre::CULL_NONE);
}
void CompoundDisplay::reset()
{
  MFDClass::reset();
  last_message_.reset();
  if (terrain_)
    terrain_->clear();
  if (occupied_)
    occupied_->clear();
}
void CompoundDisplay::redraw()
{
  if (last_message_ && terrain_)
    processMessage(last_message_);
}
void CompoundDisplay::processMessage(
    clearing_robot_plugins::msg::CompoundLocalMap::ConstSharedPtr msg)
{
  last_message_ = msg;
  terrain_->clear();
  occupied_->clear();
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation))
  {
    setStatus(rviz_common::properties::StatusProperty::Error, "Transform",
              "复合地图时间对应的坐标变换不可用");
    return;
  }
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
  grid_map::GridMap map;
  if (!grid_map::GridMapRosConverter::fromMessage(msg->terrain, map))
  {
    setStatus(rviz_common::properties::StatusProperty::Error, "Terrain", "无法解析地形地图");
    return;
  }
  setStatus(rviz_common::properties::StatusProperty::Ok, "Transform", "已使用消息时间变换");
  const auto height = height_layer_->getStdString(), color = color_layer_->getStdString();
  if (show_terrain_->getBool() && map.exists(height) &&
      (terrain_mode_->getOptionInt() != 0 || map.exists(color)))
  {
    terrain_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
    const float h = map.getResolution() / 2.0;
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
    {
      const float z = map.at(height, *it);
      if (!std::isfinite(z))
        continue;
      const float scalar = terrain_mode_->getOptionInt() == 0 ? map.at(color, *it) : 0;
      if (!std::isfinite(scalar))
        continue;
      grid_map::Position p;
      map.getPosition(*it, p);
      auto c = terrain_mode_->getOptionInt() == 0
                   ? ramp(scalar, scalar_min_->getFloat(), scalar_max_->getFloat(),
                          terrain_alpha_->getFloat(), color_map_->getOptionInt())
                   : terrain_color_->getOgreColor();
      c.a = terrain_alpha_->getFloat();
      quad(terrain_, {float(p.x() - h), float(p.y() - h), z},
           {float(p.x() + h), float(p.y() - h), z}, {float(p.x() + h), float(p.y() + h), z},
           {float(p.x() - h), float(p.y() + h), z}, c);
    }
    terrain_->end();
    setStatus(rviz_common::properties::StatusProperty::Ok, "Terrain", "地形图层有效");
  }
  else if (show_terrain_->getBool())
  {
    setStatus(rviz_common::properties::StatusProperty::Error, "Terrain",
              "指定的高度或颜色图层不存在");
  }
  if (show_occupied_->getBool())
  {
    std::unique_ptr<octomap::AbstractOcTree> raw(octomap_msgs::msgToMap(msg->occupancy));
    auto *tree = dynamic_cast<octomap::OcTree *>(raw.get());
    if (tree)
    {
      occupied_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
      const auto center = map.getPosition();
      const auto length = map.getLength();
      const double xmin = center.x() - length.x() / 2, xmax = center.x() + length.x() / 2;
      const double ymin = center.y() - length.y() / 2, ymax = center.y() + length.y() / 2;
      // 较低显示深度会绘制较大的父体素。Z 仍限制在发布障碍的原始体素范围内。
      double z_min = std::numeric_limits<double>::infinity();
      double z_max = -std::numeric_limits<double>::infinity();
      for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it)
      {
        if (tree->isNodeOccupied(*it))
        {
          const double half = it.getSize() / 2.0;
          z_min = std::min(z_min, it.getZ() - half);
          z_max = std::max(z_max, it.getZ() + half);
        }
      }
      const auto depth =
          static_cast<unsigned char>(std::min<int>(depth_->getInt(), tree->getTreeDepth()));
      for (auto it = tree->begin_leafs(depth); it != tree->end_leafs(); ++it)
      {
        if (!tree->isNodeOccupied(*it))
          continue;
        const double h = it.getSize() / 2;
        const float x0 = std::max(xmin, it.getX() - h), x1 = std::min(xmax, it.getX() + h);
        const float y0 = std::max(ymin, it.getY() - h), y1 = std::min(ymax, it.getY() + h);
        const float z0 = std::max(z_min, it.getZ() - h);
        const float z1 = std::min(z_max, it.getZ() + h);
        if (x0 >= x1 || y0 >= y1 || z0 >= z1)
        {
          continue;
        }
        auto c = voxel_mode_->getOptionInt() == 1
                     ? ramp(it.getZ(), voxel_min_->getFloat(), voxel_max_->getFloat(),
                            voxel_alpha_->getFloat(), color_map_->getOptionInt())
                     : voxel_color_->getOgreColor();
        c.a = voxel_alpha_->getFloat();
        box(occupied_, x0, x1, y0, y1, z0, z1, c);
      }
      occupied_->end();
      setStatus(rviz_common::properties::StatusProperty::Ok, "Occupancy", "占据地图有效");
    }
    else
      setStatus(rviz_common::properties::StatusProperty::Error, "Occupancy", "无法解析 OcTree");
  }
  context_->queueRender();
}
}  // namespace clearing_robot
PLUGINLIB_EXPORT_CLASS(clearing_robot::CompoundDisplay, rviz_common::Display)
