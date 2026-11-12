// Stamped commands retain their execution protocol. Framework Twist requests
// are sampled on physical clock ticks and checked against the committed state.
#include "clearing_robot_plugins/motion_limits.hpp"
#include "multimap_nav_utils/control/jerk_limited_axis.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <multimap_nav_msgs/srv/get_selected_navigation_mode.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <cstdint>

class CommandGuard : public rclcpp_lifecycle::LifecycleNode {
  using Axis=multimap_nav_utils::JerkLimitedAxis;
  using Scene=clearing_robot_plugins::Scene;
  using Pose=clearing_robot_plugins::Pose;
 public:
  CommandGuard():LifecycleNode("clearing_velocity_governor") {
    // Declare overrides first; declaration's default cannot replace a YAML "auto" override.
    declare_parameter("scene.map_mode",std::string("auto"));
    timeout_=declare_parameter("command_timeout",.25);
    wall_timeout_=declare_parameter("wall_timeout",3.);initial_dt_=declare_parameter("initial_control_interval",.05);last_interval_=initial_dt_;
    output_=create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",1);output_->on_activate();
    applied_=create_publisher<geometry_msgs::msg::TwistStamped>("/clearing/cmd_vel_smooth",10);applied_->on_activate();
    committed_=create_publisher<geometry_msgs::msg::TwistStamped>("/clearing/accepted_command",10);committed_->on_activate();
    odom_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){feedback_=m->twist.twist;feedback_stamp_=rclcpp::Time(m->header.stamp).seconds();});
    status_=create_publisher<std_msgs::msg::String>("/clearing/safety_status",10);status_->on_activate();
    selector_=create_client<multimap_nav_msgs::srv::GetSelectedNavigationMode>("/navigation_mode_selector_server/get_selected_navigation_mode");
    input_=create_subscription<geometry_msgs::msg::TwistStamped>("/clearing/cmd_vel_request",1,[this](geometry_msgs::msg::TwistStamped::ConstSharedPtr message){receive(*message);publishStatus();});
    raw_input_=create_subscription<geometry_msgs::msg::Twist>("/clearing/cmd_vel_raw",1,[this](geometry_msgs::msg::Twist::ConstSharedPtr m){
      geometry_msgs::msg::TwistStamped stamped;stamped.header.frame_id="base_link";stamped.header.stamp=now();stamped.twist=*m;
      receiveRaw(stamped);publishStatus();
    });
    blocked_=create_subscription<std_msgs::msg::Bool>("/planning/path_blocked",rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::Bool::ConstSharedPtr m){path_blocked_=m->data;});
    reset_=create_service<std_srvs::srv::Trigger>("/clearing/reset_guard",[this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
      if(std::abs(v_.velocity)>.02||std::abs(w_.velocity)>.01){response->success=false;response->message="Guard must stop before reset";return;}
      v_={};w_={};last_input_=-1e9;last_commit_=-1;last_interval_=initial_dt_;raw_active_=false;raw_pending_=false;requested_=false;requested_motion_=false;bad_=false;target_=geometry_msgs::msg::Twist();
      reason_.clear();decision_="none";reset_stamp_=now().seconds();++reset_sequence_;++commit_sequence_;
      response->success=true;response->message="Cleared stopped command history; live scene remains monitored";
    });
  }
  void start(){timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{update();});}
 private:
  bool certificate(double velocity,double omega,double acceleration,double yaw_acceleration) {
    if(!scene_||!scene_->healthy())return false;
    Pose p;
    try{p=scene_->currentPose();}catch(const tf2::TransformException&){return false;}
    auto data=scene_->snapshot();Axis va{velocity,acceleration},wa{omega,yaw_acceleration};
    if(now().seconds()-feedback_stamp_<.75){
      if(std::abs(feedback_.linear.x)>std::abs(va.velocity))va.velocity=feedback_.linear.x;
      if(std::abs(feedback_.angular.z)>std::abs(wa.velocity))wa.velocity=feedback_.angular.z;
    }
    for(int i=0;i<100;++i){
      va.advance(i<3?velocity:0.,limits_.acceleration,limits_.jerk,.1);
      wa.advance(i<3?omega:0.,limits_.yaw_acceleration,limits_.yaw_jerk,.1);
      Pose q=p;q.x+=va.velocity*std::cos(p.yaw+wa.velocity*.05)*.1;
      q.y+=va.velocity*std::sin(p.yaw+wa.velocity*.05)*.1;q.yaw=clearing_robot_plugins::wrap(p.yaw+wa.velocity*.1);
      if(!scene_->swept(p,q,data))return false;
      p=q;
      if(i>3&&std::abs(va.velocity)+std::abs(wa.velocity)+std::abs(va.acceleration)+std::abs(wa.acceleration)<1e-5)break;
    }
    return true;
  }
  void receiveRaw(const geometry_msgs::msg::TwistStamped& message){
    // Twist has no generation timestamp or control interval. Its arrival is a
    // new target, not a committed physical step; repeated arrivals while the
    // simulator is paused must not integrate velocity or advance its history.
    raw_active_=true;raw_pending_=true;
    last_requested_=message.twist;last_request_=now().seconds();
    received_wall_=std::chrono::steady_clock::now();requested_=true;
    const auto& q=message.twist;
    requested_motion_=!std::isfinite(q.linear.x+q.angular.z)||std::abs(q.linear.x)+std::abs(q.angular.z)>1e-6;
    if(q.linear.y!=0||q.linear.z!=0||q.angular.x!=0||q.angular.y!=0||
       !limits_.admissible(q.linear.x,q.angular.z,0.,0.,true)){
      bad_=true;decision_="rejected";reason_="invalid_or_out_of_range_command";return;
    }
    target_=q;bad_=false;decision_="pending";reason_.clear();
  }
  void receive(const geometry_msgs::msg::TwistStamped& message){
    const double stamp=rclcpp::Time(message.header.stamp).seconds(),time=now().seconds();
    if(stamp<reset_stamp_-1e-7)return;  // A delayed command from the preceding episode is not new input.
    raw_active_=false;raw_pending_=false;
    last_requested_=message.twist;last_request_=stamp;decision_="rejected";requested_=true;received_wall_=std::chrono::steady_clock::now();
    const auto& q=message.twist;requested_motion_=!std::isfinite(q.linear.x+q.angular.z)||std::abs(q.linear.x)+std::abs(q.angular.z)>1e-6;
    if(message.header.frame_id!="base_link"||!std::isfinite(q.linear.x+q.angular.z)||q.linear.y!=0||q.linear.z!=0||q.angular.x!=0||q.angular.y!=0||std::abs(time-stamp)>timeout_){bad_=true;reason_="invalid_or_stale_command";return;}
    // A paused-simulator keepalive has exactly the same stamp and value.
    if(std::abs(stamp-last_input_)<1e-8){
      if(std::abs(q.linear.x-target_.linear.x)+std::abs(q.angular.z-target_.angular.z)>1e-8){bad_=true;reason_="changed_command_without_physical_step";}
      decision_=bad_?"rejected":"accepted";return;
    }
    const double dt=last_commit_<0?initial_dt_:stamp-last_commit_;
    const double jerk_dt=.5*(last_interval_+dt);
    if(dt<=0||dt>.75){bad_=true;reason_="invalid_control_interval";return;}
    double a=(q.linear.x-v_.velocity)/dt,aw=(q.angular.z-w_.velocity)/dt;
    if(path_blocked_){bad_=true;reason_="global_path_blocked";return;}
    if(!scene_||!scene_->healthy()){bad_=true;reason_="unhealthy_scene";return;}
    if(!limits_.admissible(q.linear.x,q.angular.z,a,aw,true)||
       std::abs(a-v_.acceleration)>limits_.jerk*jerk_dt+1e-5||std::abs(aw-w_.acceleration)>limits_.yaw_jerk*jerk_dt+1e-5){bad_=true;reason_="command_constraint";return;}
    if(!certificate(q.linear.x,q.angular.z,a,aw)){bad_=true;reason_="unsafe_stopping_envelope";return;}
    decision_="accepted";target_=q;v_.velocity=q.linear.x;w_.velocity=q.angular.z;v_.acceleration=a;w_.acceleration=aw;
    last_input_=stamp;last_commit_=stamp;last_interval_=dt;bad_=false;reason_="";++commit_sequence_;output_->publish(q);committed_->publish(message);
  }
  void update(){
    if(get_publishers_info_by_topic("/cmd_vel").size()>1){RCLCPP_ERROR(get_logger(),"Multiple final command owners; use an isolated ROS_DOMAIN_ID");rclcpp::shutdown();return;}
    const double time=now().seconds();double dt=last_update_<0?0.:std::clamp(time-last_update_,0.,.1);last_update_=time;
    if(!scene_&&!pending_&&selector_->service_is_ready()){
      pending_=true;selector_->async_send_request(std::make_shared<multimap_nav_msgs::srv::GetSelectedNavigationMode::Request>(),[this](rclcpp::Client<multimap_nav_msgs::srv::GetSelectedNavigationMode>::SharedFuture future){
        pending_=false;
        try{
          auto response=future.get();if(!response->ready||!response->success)return;
          const auto result=set_parameter(rclcpp::Parameter("scene.map_mode",response->selected_mode));
          if(!result.successful)throw std::runtime_error("Selected map parameter rejected: "+result.reason);
          auto scene=Scene::create(shared_from_this());
          auto limits=clearing_robot_plugins::MotionLimits::load(shared_from_this(),scene->model);
          scene_=std::move(scene);limits_=limits;
          RCLCPP_INFO(get_logger(),"Safety scene configured for selected map: %s",response->selected_mode.c_str());
        }catch(const std::exception& error){
          RCLCPP_ERROR(get_logger(),"Safety scene initialization failed: %s",error.what());
          v_={};w_={};geometry_msgs::msg::Twist stopped;output_->publish(stopped);rclcpp::shutdown();
        }
      });
    }
    const double wall_age=std::chrono::duration<double>(std::chrono::steady_clock::now()-received_wall_).count();
    bool fault=bad_||path_blocked_||!scene_||!scene_->healthy()||
        time-(raw_active_?last_request_:last_input_)>timeout_||wall_age>wall_timeout_;
    bool raw_committed=false;
    if(!fault&&raw_active_&&raw_pending_&&(last_commit_<0||time>last_commit_+1e-8)){
      raw_pending_=false;
      const double sample_dt=last_commit_<0?initial_dt_:time-last_commit_;
      // The planner already produces a limited command. Validate its sampled
      // increment once; a second S-curve here would delay braking and tracking.
      Axis next_v{target_.linear.x,(target_.linear.x-v_.velocity)/sample_dt};
      Axis next_w{target_.angular.z,(target_.angular.z-w_.velocity)/sample_dt};
      const double jerk_dt=.5*(last_interval_+sample_dt);
      if(sample_dt<=0||sample_dt>.75||
         !limits_.admissible(next_v.velocity,next_w.velocity,next_v.acceleration,next_w.acceleration,true)||
         std::abs(next_v.acceleration-v_.acceleration)>limits_.jerk*jerk_dt+1e-5||
         std::abs(next_w.acceleration-w_.acceleration)>limits_.yaw_jerk*jerk_dt+1e-5){
        bad_=true;reason_="command_constraint";
      }else if(!certificate(next_v.velocity,next_w.velocity,next_v.acceleration,next_w.acceleration)){
        bad_=true;reason_="unsafe_stopping_envelope";
      }else{
        v_=next_v;w_=next_w;last_input_=last_request_;last_commit_=time;last_interval_=sample_dt;
        decision_="accepted";reason_.clear();++commit_sequence_;raw_committed=true;
      }
      fault=bad_;
      if(fault)decision_="rejected";
    }
    if(fault){
      if(!bad_)reason_=path_blocked_?"global_path_blocked":!scene_?"waiting_for_scene":!scene_->healthy()?"unhealthy_scene":"command_watchdog";
      v_.advance(0.,limits_.acceleration,limits_.jerk,dt);w_.advance(0.,limits_.yaw_acceleration,limits_.yaw_jerk,dt);
      if(scene_){try{if(!scene_->safe(scene_->currentPose(),scene_->snapshot())){v_={};w_={};}}catch(const tf2::TransformException&) {}}
    }
    geometry_msgs::msg::Twist command;command.linear.x=v_.velocity;command.angular.z=w_.velocity;output_->publish(command);
    geometry_msgs::msg::TwistStamped stamped;stamped.header.frame_id="base_link";stamped.header.stamp=now();stamped.twist=command;applied_->publish(stamped);
    if(raw_committed)committed_->publish(stamped);
    if(fault&&dt>0&&requested_){committed_->publish(stamped);last_commit_=time;last_interval_=dt;++commit_sequence_;}
    publishStatus();
  }
  void publishStatus(){
    const double time=now().seconds();
    const bool planned=path_blocked_ && reason_=="global_path_blocked";
    const bool intervention=!planned && (bad_ || (requested_ && reason_!="" && requested_motion_));
    std_msgs::msg::String status;
    std::ostringstream stream;
    stream << "{\"ready\":" << (scene_&&scene_->healthy()?"true":"false")
           << ",\"intervention\":" << (intervention?"true":"false")
           << ",\"planned_stop\":" << (planned?"true":"false")
           << ",\"request_stamp\":" << std::setprecision(17) << last_request_
           << ",\"accepted_stamp\":" << last_input_
           << ",\"decision\":\"" << decision_ << "\",\"command_protocol\":3,\"reset_sequence\":" << reset_sequence_
           << ",\"status_sequence\":" << ++status_sequence_
           << ",\"commit_sequence\":" << commit_sequence_
           << ",\"status_stamp\":" << time
           << ",\"commit_stamp\":" << last_commit_
           << ",\"command_interval\":" << last_interval_
           << ",\"requested_command\":[" << last_requested_.linear.x << "," << last_requested_.angular.z << "]"
           << ",\"command\":[" << v_.velocity << "," << w_.velocity << "]"
           << ",\"command_acceleration\":[" << v_.acceleration << "," << w_.acceleration << "]"
           << ",\"reason\":\"" << reason_ << "\"}";
    status.data=stream.str();status_->publish(status);
  }
  std::shared_ptr<Scene> scene_;clearing_robot_plugins::MotionLimits limits_;Axis v_,w_;
  geometry_msgs::msg::Twist target_,last_requested_;bool bad_{false},requested_{false},requested_motion_{false},pending_{false},raw_active_{false},raw_pending_{false};std::string decision_{"none"};
  std::string reason_;
  double initial_dt_{.05},last_interval_{.05},timeout_{.25},wall_timeout_{3.},last_input_{-1e9},last_update_{-1},last_commit_{-1},last_request_{-1e9},feedback_stamp_{-1e9};
  double reset_stamp_{-1e9};std::uint64_t reset_sequence_{0},commit_sequence_{0},status_sequence_{0};
  geometry_msgs::msg::Twist feedback_;
  std::chrono::steady_clock::time_point received_wall_{std::chrono::steady_clock::now()};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr output_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr applied_,committed_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_;
  bool path_blocked_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr blocked_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_input_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr input_;
  rclcpp::Client<multimap_nav_msgs::srv::GetSelectedNavigationMode>::SharedPtr selector_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);auto node=std::make_shared<CommandGuard>();node->start();rclcpp::spin(node->get_node_base_interface());rclcpp::shutdown();}
