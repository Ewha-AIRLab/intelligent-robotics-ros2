// Author:  Daeun Song <songd@ewha.ac.kr>

#include <cmath>
#include <map>
#include <string>
#include <chrono>

#include <sdf/Actor.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Actor.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace dynamic_actor
{

class ActorPosePublisher
  : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPostUpdate
{
public:
  void Configure(
    const ignition::gazebo::Entity &,
    const std::shared_ptr<const sdf::Element> &,
    ignition::gazebo::EntityComponentManager &,
    ignition::gazebo::EventManager &) override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("actor_pose_publisher");
  }

  void PostUpdate(
    const ignition::gazebo::UpdateInfo &info,
    const ignition::gazebo::EntityComponentManager &ecm) override
  {
    if (info.paused) return;

    const double simTimeSec =
      std::chrono::duration<double>(info.simTime).count();

    ecm.Each<ignition::gazebo::components::Actor,
             ignition::gazebo::components::Name>(
      [&](const ignition::gazebo::Entity &entity,
          const ignition::gazebo::components::Actor *actorComp,
          const ignition::gazebo::components::Name *name) -> bool
      {
        ignition::math::Vector3d pos;
        ignition::math::Quaterniond rot;

        // Source 1: WorldPose written by TrajectoryActorPlugin
        auto *worldPose =
          ecm.Component<ignition::gazebo::components::WorldPose>(entity);
        if (worldPose) {
          pos = worldPose->Data().Pos();
          rot = worldPose->Data().Rot();
        } else {
          // Source 2: SDF trajectory interpolation (script-driven actors)
          const sdf::Actor &sdfActor = actorComp->Data();
          if (sdfActor.TrajectoryCount() == 0) return true;

          const sdf::Trajectory *traj = sdfActor.TrajectoryByIndex(0);
          const uint64_t wpCount = traj->WaypointCount();
          if (wpCount < 2) return true;

          const double loopDuration =
            traj->WaypointByIndex(wpCount - 1)->Time();
          if (loopDuration <= 0.0) return true;

          double t = simTimeSec - sdfActor.ScriptDelayStart();
          if (t < 0.0) return true;
          if (sdfActor.ScriptLoop())
            t = std::fmod(t, loopDuration);
          else
            t = std::min(t, loopDuration);

          uint64_t idxA = 0;
          for (uint64_t i = 0; i + 1 < wpCount; ++i) {
            if (traj->WaypointByIndex(i + 1)->Time() >= t) {
              idxA = i;
              break;
            }
          }
          const uint64_t idxB = idxA + 1 < wpCount ? idxA + 1 : idxA;

          const sdf::Waypoint *wpA = traj->WaypointByIndex(idxA);
          const sdf::Waypoint *wpB = traj->WaypointByIndex(idxB);
          const double dt    = wpB->Time() - wpA->Time();
          const double alpha = (dt > 1e-9) ? (t - wpA->Time()) / dt : 0.0;

          const auto pA = wpA->Pose();
          const auto pB = wpB->Pose();
          pos = pA.Pos() + alpha * (pB.Pos() - pA.Pos());
          rot = ignition::math::Quaterniond::Slerp(
            alpha, pA.Rot(), pB.Rot(), true);
        }

        // Lazily create publisher for this actor
        const std::string &actorName = name->Data();
        if (publishers_.find(actorName) == publishers_.end()) {
          publishers_[actorName] =
            node_->create_publisher<geometry_msgs::msg::PoseStamped>(
              "/actor/" + actorName + "/pose", 10);
        }

        const auto totalNs =
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            info.simTime).count();

        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id      = "world";
        msg.header.stamp.sec     = static_cast<int32_t>(totalNs / 1000000000);
        msg.header.stamp.nanosec = static_cast<uint32_t>(totalNs % 1000000000);
        msg.pose.position.x      = pos.X();
        msg.pose.position.y      = pos.Y();
        msg.pose.position.z      = pos.Z();
        msg.pose.orientation.x   = rot.X();
        msg.pose.orientation.y   = rot.Y();
        msg.pose.orientation.z   = rot.Z();
        msg.pose.orientation.w   = rot.W();

        publishers_[actorName]->publish(msg);
        return true;
      });

    if (rclcpp::ok())
      rclcpp::spin_some(node_);
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::map<std::string,
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> publishers_;
};

}  // namespace dynamic_actor

IGNITION_ADD_PLUGIN(
  dynamic_actor::ActorPosePublisher,
  ignition::gazebo::System,
  dynamic_actor::ActorPosePublisher::ISystemConfigure,
  dynamic_actor::ActorPosePublisher::ISystemPostUpdate)