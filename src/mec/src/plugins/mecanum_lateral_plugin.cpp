#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace gazebo
{
  class MecanumLateralPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      model_ = _model;

      if (_sdf->HasElement("body_name"))
        body_name_ = _sdf->Get<std::string>("body_name");

      if (_sdf->HasElement("command_topic"))
        topic_ = _sdf->Get<std::string>("command_topic");

      node_ = rclcpp::Node::make_shared("mecanum_lateral_plugin_node");
      sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
          topic_, 10, std::bind(&MecanumLateralPlugin::OnCmdVel, this, std::placeholders::_1));

      update_connection_ = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MecanumLateralPlugin::OnUpdate, this));
    }

    void OnCmdVel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
      vy_ = msg->twist.linear.y;  // lateral (fake)
      // vx و vtheta لا نلمسهم هنا → يعتمدوا على الفيزياء
    }

    void OnUpdate()
    {
      auto body = model_->GetLink(body_name_);
      if (!body) return;

      double dt = model_->GetWorld()->Physics()->GetMaxStepSize();
      auto pose = body->WorldPose();

      // حركة lateral بالنسبة لمقدمة الروبوت
      double yaw = pose.Rot().Yaw();
      pose.Pos().X() += vy_ * dt * sin(yaw);  // lateral يؤثر على X عالمي فقط بحسب yaw
      pose.Pos().Y() += vy_ * dt * cos(yaw);  // lateral يؤثر على Y عالمي فقط بحسب yaw

      body->SetWorldPose(pose);
    }

  private:
    physics::ModelPtr model_;
    std::string body_name_;
    std::string topic_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
    event::ConnectionPtr update_connection_;

    double vy_ = 0.0;
  };

  GZ_REGISTER_MODEL_PLUGIN(MecanumLateralPlugin)
}
