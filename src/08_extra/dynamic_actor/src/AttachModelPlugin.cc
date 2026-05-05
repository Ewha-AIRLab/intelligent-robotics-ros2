// Author:  Daeun Song <songd@ewha.ac.kr>
//
// AttachModelPlugin — Fortress equivalent of Gazebo Classic libAttachModelPlugin.so
//
// Teleports a ghost collision model to the actor's current pose every tick so
// that Gazebo LiDAR / contact sensors can detect the actor.
//
// Pose source (in priority order):
//   1. components::WorldPose on the actor  — set by TrajectoryActorPlugin
//   2. SDF trajectory interpolation        — fallback for <script>-driven actors
//
// SDF usage (inside <actor>):
//   <plugin filename="AttachModelPlugin" name="dynamic_actor::AttachModelPlugin">
//     <model_name>walking_actor_collision</model_name>
//   </plugin>

#include <cmath>
#include <string>
#include <chrono>

#include <sdf/Actor.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Actor.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

namespace dynamic_actor
{

class AttachModelPlugin
  : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
{
public:
  void Configure(
    const ignition::gazebo::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    ignition::gazebo::EntityComponentManager &,
    ignition::gazebo::EventManager &) override
  {
    actorEntity_ = entity;

    if (!sdf->HasElement("model_name")) {
      ignerr << "[AttachModelPlugin] Missing required <model_name> element.\n";
      return;
    }
    modelName_ = sdf->Get<std::string>("model_name");
  }

  void PreUpdate(
    const ignition::gazebo::UpdateInfo &info,
    ignition::gazebo::EntityComponentManager &ecm) override
  {
    if (info.paused || modelName_.empty()) return;

    // Lazily find the ghost model entity by name
    if (ghostEntity_ == ignition::gazebo::kNullEntity) {
      ecm.Each<ignition::gazebo::components::Model,
               ignition::gazebo::components::Name>(
        [&](const ignition::gazebo::Entity &e,
            const ignition::gazebo::components::Model *,
            const ignition::gazebo::components::Name *name) -> bool
        {
          if (name->Data() == modelName_) {
            ghostEntity_ = e;
            return false;
          }
          return true;
        });
      if (ghostEntity_ == ignition::gazebo::kNullEntity) return;
    }

    // --- Pose source 1: WorldPose written by TrajectoryActorPlugin ---
    auto *worldPose =
      ecm.Component<ignition::gazebo::components::WorldPose>(actorEntity_);
    if (worldPose) {
      setGhostPose(ecm, worldPose->Data());
      return;
    }

    // --- Pose source 2: SDF trajectory interpolation (script-driven actors) ---
    auto *actorComp =
      ecm.Component<ignition::gazebo::components::Actor>(actorEntity_);
    if (!actorComp) return;

    const sdf::Actor &sdfActor = actorComp->Data();
    if (sdfActor.TrajectoryCount() == 0) return;

    const sdf::Trajectory *traj = sdfActor.TrajectoryByIndex(0);
    const uint64_t wpCount = traj->WaypointCount();
    if (wpCount < 2) return;

    const double loopDuration = traj->WaypointByIndex(wpCount - 1)->Time();
    if (loopDuration <= 0.0) return;

    double t =
      std::chrono::duration<double>(info.simTime).count()
      - sdfActor.ScriptDelayStart();
    if (t < 0.0) return;
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
    const uint64_t idxB = (idxA + 1 < wpCount) ? idxA + 1 : idxA;

    const auto *wpA = traj->WaypointByIndex(idxA);
    const auto *wpB = traj->WaypointByIndex(idxB);
    const double dt    = wpB->Time() - wpA->Time();
    const double alpha = (dt > 1e-9) ? (t - wpA->Time()) / dt : 0.0;

    const auto pA = wpA->Pose();
    const auto pB = wpB->Pose();

    const ignition::math::Pose3d interpolated(
      pA.Pos() + alpha * (pB.Pos() - pA.Pos()),
      ignition::math::Quaterniond::Slerp(alpha, pA.Rot(), pB.Rot(), true));

    setGhostPose(ecm, interpolated);
  }

private:
  void setGhostPose(
    ignition::gazebo::EntityComponentManager &ecm,
    const ignition::math::Pose3d &pose)
  {
    auto *cmd =
      ecm.Component<ignition::gazebo::components::WorldPoseCmd>(ghostEntity_);
    if (!cmd)
      ecm.CreateComponent(ghostEntity_,
        ignition::gazebo::components::WorldPoseCmd(pose));
    else
      *cmd = ignition::gazebo::components::WorldPoseCmd(pose);
  }

  ignition::gazebo::Entity actorEntity_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity ghostEntity_{ignition::gazebo::kNullEntity};
  std::string modelName_;
};

}  // namespace dynamic_actor

IGNITION_ADD_PLUGIN(
  dynamic_actor::AttachModelPlugin,
  ignition::gazebo::System,
  dynamic_actor::AttachModelPlugin::ISystemConfigure,
  dynamic_actor::AttachModelPlugin::ISystemPreUpdate)
