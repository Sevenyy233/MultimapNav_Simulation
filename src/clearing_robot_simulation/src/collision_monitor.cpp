#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/bool.hpp>

namespace clearing_robot
{

/** @brief 采集 Gazebo 的接触事件，区分履带正常支承和实际碰撞。 */
class CollisionMonitor : public gazebo::WorldPlugin
{
public:
  /** @brief 注册物理更新回调和碰撞话题；不控制物理步进。
   * @param world 当前训练或评估场景。
   * @param config 当前 world 中的插件配置，可覆盖 entity 名称。
   */
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr config) override
  {
    world_ = std::move(world);
    entity_ = config->Get<std::string>("entity", "clearing_robot").first;
    node_ = gazebo_ros::Node::Get(config);
    contacts_ = world_->Physics()->GetContactManager();
    contacts_->SetNeverDropContacts(true);
    collision_ = node_->create_publisher<std_msgs::msg::Bool>("/clearing/contact_collision", 5);
    update_ = gazebo::event::Events::ConnectWorldUpdateEnd([this]() {sampleContacts();});
  }

private:
  /** @brief 过滤自碰撞和履带在地面、坡面的正常支承触点。 */
  bool collision(const gazebo::physics::Contact & contact) const
  {
    if (contact.count <= 0 || contact.collision1 == nullptr || contact.collision2 == nullptr) {
      return false;
    }
    const auto first = contact.collision1->GetLink();
    const auto second = contact.collision2->GetLink();
    if (!first || !second) {
      return false;
    }
    const bool first_robot = first->GetModel()->GetName() == entity_;
    const bool second_robot = second->GetModel()->GetName() == entity_;
    if (first_robot == second_robot) {
      return false;
    }
    const auto & robot_link = first_robot ? first : second;
    const auto & external_link = first_robot ? second : first;
    const auto name = external_link->GetModel()->GetName();
    const bool support = name.rfind("ground_", 0) == 0 || name.rfind("terrain_", 0) == 0;
    const bool wheel = robot_link->GetName().find("_wheel_") != std::string::npos;
    if (!support || !wheel) {
      return true;
    }
    // 轮缘水平碰撞（例如坑壁或台阶）与承载重量的竖向接触不同。
    for (int index = 0; index < contact.count; ++index) {
      const auto & normal = contact.normals[index];
      if (std::abs(normal.Z()) < 0.25 && contact.depths[index] > 0.007) {
        return true;
      }
    }
    return false;
  }

  /** @brief 锁存短触点 0.5 个仿真秒，在 20 Hz 发布碰撞标志。 */
  void sampleContacts()
  {
    const double now = world_->SimTime().Double();
    if (now < last_publish_) {
      last_publish_ = -1.0;
      hold_until_ = -1.0;
    }
    const auto count = contacts_->GetContactCount();
    const auto & contacts = contacts_->GetContacts();
    for (unsigned index = 0; index < count && index < contacts.size(); ++index) {
      if (contacts[index] && collision(*contacts[index])) {
        hold_until_ = now + 0.5;
        break;
      }
    }
    if (now - last_publish_ >= 0.05) {
      std_msgs::msg::Bool message;
      message.data = now <= hold_until_;
      collision_->publish(message);
      last_publish_ = now;
    }
  }

  std::string entity_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::ContactManager * contacts_{nullptr};
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_;
  gazebo::event::ConnectionPtr update_;
  double last_publish_{-1.0};
  double hold_until_{-1.0};
};

GZ_REGISTER_WORLD_PLUGIN(CollisionMonitor)

}  // namespace clearing_robot
