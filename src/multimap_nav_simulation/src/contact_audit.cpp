#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <cctype>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/string.hpp>
#include "multimap_nav_simulation/contact_classification.hpp"

namespace gazebo
{
// Evaluation only. Captures every physics-step contact, publishing aggregates
// at 10 Hz. Nothing from this plugin is an input to navigation or TF.
class ContactAudit : public WorldPlugin
{
public:
  void Load(physics::WorldPtr world,sdf::ElementPtr sdf) override
  {
    world_=std::move(world);
    robot_=sdf->HasElement("robot_model")?sdf->Get<std::string>("robot_model"):"tracked_robot";
    node_=gazebo_ros::Node::Get(sdf);
    publisher_=node_->create_publisher<std_msgs::msg::String>("/evaluation/contact_state",20);
    manager_=world_->Physics()->GetContactManager();
    manager_->SetNeverDropContacts(true);
    connection_=event::Events::ConnectWorldUpdateEnd(std::bind(&ContactAudit::update,this));
  }
private:
  void update()
  {
    const double now=world_->SimTime().Double();
    if(now<last_publish_){last_seen_.clear();episodes_=0;last_publish_=now;}
    const auto & contacts=manager_->GetContacts();
    std::set<std::string> hazards;
    for(unsigned i=0;i<manager_->GetContactCount();++i){
      const auto * c=contacts[i];
      if(!c||!c->collision1||!c->collision2)continue;
      const auto a=c->collision1->GetScopedName(),b=c->collision2->GetScopedName();
      const bool first=a.find(robot_+"::")==0,second=b.find(robot_+"::")==0;
      if(first==second)continue; // No robot contact, or internal self-contact.
      const std::string other=first?b:a;
      for(int j=0;j<c->count;++j){
        const double nz=(first?1.0:-1.0)*c->normals[j].Z();
        normal_min_=std::min(normal_min_,nz);normal_max_=std::max(normal_max_,nz);
        // An upward reaction on the robot is a support contact on the
        // traversable surface. Side/ceiling contacts are obstacle contacts.
        if(multimap_nav_simulation::isSupportContact(other,nz)){
          ++support_points_;
          if(nz<std::cos(M_PI/4.0))++support_edge_points_;
          continue;
        }
        ++obstacle_points_;hazards.insert(other);active_.insert(other);
        obstacle_normal_min_=std::min(obstacle_normal_min_,nz);
        obstacle_normal_max_=std::max(obstacle_normal_max_,nz);
        penetration_=std::max(penetration_,c->depths[j]);
      }
    }
    for(const auto & other:hazards){
      const auto previous=last_seen_.find(other);
      if(previous==last_seen_.end()||now-previous->second>.20)++episodes_;
      last_seen_[other]=now;
    }
    if(now-last_publish_<.10)return;
    std::ostringstream json;json.precision(12);
    json << "{\"stamp\":" << now << ",\"collision_episodes\":" << episodes_
         << ",\"classification_version\":4"
         << ",\"obstacle_contact_points\":" << obstacle_points_
         << ",\"support_contact_points\":" << support_points_
         << ",\"support_edge_contact_points\":" << support_edge_points_
         << ",\"max_penetration_m\":" << penetration_ << ",\"normal_z_min\":";
    if(std::isfinite(normal_min_))json<<normal_min_;else json<<"null";
    json << ",\"normal_z_max\":";
    if(std::isfinite(normal_max_))json<<normal_max_;else json<<"null";
    json << ",\"obstacle_normal_z_min\":";
    if(std::isfinite(obstacle_normal_min_))json<<obstacle_normal_min_;else json<<"null";
    json << ",\"obstacle_normal_z_max\":";
    if(std::isfinite(obstacle_normal_max_))json<<obstacle_normal_max_;else json<<"null";
    json << ",\"active_obstacle_colliders\":[";
    bool comma=false;for(const auto & name:active_){if(comma)json<<',';json<<std::quoted(name);comma=true;}
    json << "]}";
    std_msgs::msg::String msg;msg.data=json.str();publisher_->publish(msg);
    last_publish_=now;support_points_=obstacle_points_=support_edge_points_=0;penetration_=0.;active_.clear();
    normal_min_=INFINITY;normal_max_=-INFINITY;
    obstacle_normal_min_=INFINITY;obstacle_normal_max_=-INFINITY;
  }
  physics::WorldPtr world_;
  physics::ContactManager * manager_{nullptr};
  event::ConnectionPtr connection_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string robot_;
  std::map<std::string,double> last_seen_;
  std::set<std::string> active_;
  size_t episodes_{0},support_points_{0},obstacle_points_{0},support_edge_points_{0};
  double last_publish_{0.},penetration_{0.},normal_min_{INFINITY},normal_max_{-INFINITY};
  double obstacle_normal_min_{INFINITY},obstacle_normal_max_{-INFINITY};
};
GZ_REGISTER_WORLD_PLUGIN(ContactAudit)
}
