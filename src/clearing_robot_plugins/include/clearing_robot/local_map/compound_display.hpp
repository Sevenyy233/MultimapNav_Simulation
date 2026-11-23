#ifndef CLEARING_ROBOT_COMPOUND_DISPLAY_HPP_
#define CLEARING_ROBOT_COMPOUND_DISPLAY_HPP_

#include <rviz_common/message_filter_display.hpp>

#include "clearing_robot_plugins/msg/compound_local_map.hpp"

namespace Ogre
{
class ManualObject;
}
namespace rviz_common::properties
{
class BoolProperty;
class ColorProperty;
class EnumProperty;
class FloatProperty;
class IntProperty;
class StringProperty;
}  // namespace rviz_common::properties
namespace clearing_robot
{
/** @brief 同步显示复合局部地图的地形网格与占据体素。 */
class CompoundDisplay
    : public rviz_common::MessageFilterDisplay<clearing_robot_plugins::msg::CompoundLocalMap>
{
  Q_OBJECT
 public:
  /** @brief 建立可交互的图层显示属性。 */
  CompoundDisplay();
  /** @brief 释放场景对象与材质。 */
  ~CompoundDisplay() override;

 protected:
  /** @brief 初始化场景对象。 */
  void onInitialize() override;
  /** @brief 清除缓存及显示。 */
  void reset() override;
 private Q_SLOTS:
  /** @brief 参数变化时重绘缓存消息。 */
  void redraw();

 private:
  /** @brief 根据复合消息头的时间和坐标系渲染两个图层。 */
  void processMessage(clearing_robot_plugins::msg::CompoundLocalMap::ConstSharedPtr msg) override;
  Ogre::ManualObject* terrain_ = nullptr;
  Ogre::ManualObject* occupied_ = nullptr;
  std::string material_name_;
  clearing_robot_plugins::msg::CompoundLocalMap::ConstSharedPtr last_message_;
  rviz_common::properties::BoolProperty *show_terrain_, *show_occupied_;
  rviz_common::properties::StringProperty *height_layer_, *color_layer_;
  rviz_common::properties::EnumProperty *terrain_mode_, *voxel_mode_, *color_map_;
  rviz_common::properties::ColorProperty *terrain_color_, *voxel_color_;
  rviz_common::properties::FloatProperty *scalar_min_, *scalar_max_, *terrain_alpha_, *voxel_alpha_,
      *voxel_min_, *voxel_max_;
  rviz_common::properties::IntProperty* depth_;
};
}  // namespace clearing_robot
#endif
