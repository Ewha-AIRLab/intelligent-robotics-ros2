// Author:  Daeun Song <songd@ewha.ac.kr>
//
// TrajectoryActorPlugin — Fortress equivalent of Gazebo Classic libTrajectoryActorPlugin.so
//
// Drives an actor along a series of <target> waypoints at a constant velocity.
// Uses potential-field steering to navigate around obstacles:
//   attraction toward the current target + repulsion from nearby colliders.
// Controls skeletal animation via AnimationName / AnimationTime components.
//
// The actor in SDF must have <skin> and <animation> but NO <script> block.
// The plugin owns all position and animation state.
//
// Orientation model — everything lives in <init>:
//   x y z      — world spawn position
//   roll/pitch — upright correction (e.g. 1.5708 0 for Y-up DAE → +Z up)
//   yaw        — mesh-forward correction: offset added to movement heading each tick
//                so the mesh's native front aligns with +X after roll/pitch
//
// The actor's SDF <pose> should be neutral (0 0 0  0 0 0).
// The plugin owns all pose state from the first tick.
//
// Obstacle avoidance (automatic):
//   - Only models with actual collision geometry are considered obstacles.
//   - Static models (ground, walls) are ignored automatically.
//   - <ignore> lists model names to always exclude.
//   - <obstacle_margin>: repulsion starts at this distance (metres).
//   - <repulsion_gain>:  strength of avoidance force (tune if actor clips walls).
//
// SDF usage (inside <actor>):
//   <pose>0 0 0  0 0 0</pose>  <!-- neutral; plugin takes over tick 1 -->
//   <plugin filename="TrajectoryActorPlugin" name="dynamic_actor::TrajectoryActorPlugin">
//     <init>-3 -3 0.35  1.5708 0 0</init>  <!-- spawn pos + mesh correction -->
//     <target>-3 -3 0.35  0 0 0</target>   <!-- x y z only; orientation ignored -->
//     <target> 3 -3 0.35  0 0 0</target>
//     <target> 3  3 0.35  0 0 0</target>
//     <target>-3  3 0.35  0 0 0</target>
//     <velocity>1.0</velocity>
//     <obstacle_margin>1.5</obstacle_margin>   <!-- optional -->
//     <repulsion_gain>2.0</repulsion_gain>     <!-- optional -->
//     <ignore>my_collision_cylinder</ignore>   <!-- optional, repeatable -->
//   </plugin>

#include <cmath>
#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Actor.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

namespace dynamic_actor
{

class TrajectoryActorPlugin
  : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
{
public:
  void Configure(
    const ignition::gazebo::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    ignition::gazebo::EntityComponentManager &ecm,
    ignition::gazebo::EventManager &) override
  {
    actorEntity_ = entity;

    velocity_       = sdf->Get<double>("velocity",        1.0).first;
    obstacleMargin_ = sdf->Get<double>("obstacle_margin", 1.5).first;
    repulsionGain_  = sdf->Get<double>("repulsion_gain",  2.0).first;

    auto avoidElem = sdf->FindElement("collision_avoidance");
    if (avoidElem) {
      avoidanceEnabled_ = true;
      auto ig = avoidElem->FindElement("ignore");
      while (ig) {
        ignoreNames_.push_back(ig->Get<std::string>());
        ig = ig->GetNextElement("ignore");
      }
    }

    auto elem = sdf->FindElement("target");
    while (elem) {
      targets_.push_back(elem->Get<ignition::math::Pose3d>().Pos());
      elem = elem->GetNextElement("target");
    }
    if (targets_.empty()) {
      ignerr << "[TrajectoryActorPlugin] No <target> elements found.\n";
      return;
    }

    // <init> owns both the spawn position and the mesh correction orientation.
    //   position (x y z) — where the actor starts in the world
    //   roll / pitch     — correction so that after actor <pose>, +X is front and +Z is up
    //   yaw              — ignored; heading is computed each tick from movement direction
    const auto initPose =
      sdf->Get<ignition::math::Pose3d>("init",
        ignition::math::Pose3d::Zero).first;
    currentPos_ = initPose.Pos();
    initRoll_   = initPose.Rot().Roll();
    initPitch_  = initPose.Rot().Pitch();
    initYaw_    = initPose.Rot().Yaw();

    if (!ecm.Component<ignition::gazebo::components::WorldPose>(entity))
      ecm.CreateComponent(entity, ignition::gazebo::components::WorldPose());
  }

  void PreUpdate(
    const ignition::gazebo::UpdateInfo &info,
    ignition::gazebo::EntityComponentManager &ecm) override
  {
    if (info.paused || targets_.empty()) return;

    if (!animInitialized_) {
      if (!ecm.Component<ignition::gazebo::components::AnimationName>(actorEntity_))
        ecm.CreateComponent(actorEntity_,
          ignition::gazebo::components::AnimationName("walking"));
      if (!ecm.Component<ignition::gazebo::components::AnimationTime>(actorEntity_))
        ecm.CreateComponent(actorEntity_,
          ignition::gazebo::components::AnimationTime(
            std::chrono::steady_clock::duration::zero()));
      if (!ecm.Component<ignition::gazebo::components::TrajectoryPose>(actorEntity_))
        ecm.CreateComponent(actorEntity_,
          ignition::gazebo::components::TrajectoryPose(
            ignition::math::Pose3d::Zero));
      animInitialized_ = true;
    }

    const double dt = std::chrono::duration<double>(info.dt).count();

    const auto &target = targets_[currentTarget_];
    const double tdx   = target.X() - currentPos_.X();
    const double tdy   = target.Y() - currentPos_.Y();
    const double tdist = std::sqrt(tdx * tdx + tdy * tdy);

    if (tdist < 0.3) {
      currentTarget_ = (currentTarget_ + 1) % targets_.size();
      return;
    }

    // --- Potential-field steering ---
    // Attraction: unit vector toward current target
    double fx = tdx / tdist;
    double fy = tdy / tdist;

    if (avoidanceEnabled_) {
      std::unordered_map<ignition::gazebo::Entity, ignition::math::Vector3d>
        collidableModels;
      ecm.Each<ignition::gazebo::components::Collision>(
        [&](const ignition::gazebo::Entity &colEnt,
            const ignition::gazebo::components::Collision *) -> bool
        {
          const auto linkEnt  = ecm.ParentEntity(colEnt);
          const auto modelEnt = ecm.ParentEntity(linkEnt);
          if (modelEnt == ignition::gazebo::kNullEntity) return true;

          auto *staticComp =
            ecm.Component<ignition::gazebo::components::Static>(modelEnt);
          if (staticComp && staticComp->Data()) return true;

          auto *name =
            ecm.Component<ignition::gazebo::components::Name>(modelEnt);
          if (name) {
            for (const auto &n : ignoreNames_)
              if (name->Data() == n) return true;
          }

          auto *modelPose =
            ecm.Component<ignition::gazebo::components::Pose>(modelEnt);
          if (modelPose)
            collidableModels[modelEnt] = modelPose->Data().Pos();
          return true;
        });

      for (const auto &[modelEnt, obsPos] : collidableModels) {
        const double dx = currentPos_.X() - obsPos.X();
        const double dy = currentPos_.Y() - obsPos.Y();
        const double d  = std::sqrt(dx * dx + dy * dy);
        if (d < obstacleMargin_ && d > 0.01) {
          const double strength =
            repulsionGain_ * (1.0 / d - 1.0 / obstacleMargin_) / (d * d);
          fx += strength * dx / d;
          fy += strength * dy / d;
        }
      }
    }

    // Normalise combined force to constant speed
    const double fmag = std::sqrt(fx * fx + fy * fy);
    if (fmag > 1e-3) { fx /= fmag; fy /= fmag; }

    const double step = velocity_ * dt;
    currentPos_.X() += step * fx;
    currentPos_.Y() += step * fy;
    currentPos_.Z()  = target.Z();

    // Movement heading + initYaw_ aligns the mesh forward with +X after correction.
    const double yaw = std::atan2(fy, fx) + initYaw_;
    const ignition::math::Pose3d newPose(
      currentPos_,
      ignition::math::Quaterniond(initRoll_, initPitch_, yaw));

    auto *poseComp =
      ecm.Component<ignition::gazebo::components::Pose>(actorEntity_);
    if (poseComp) {
      poseComp->Data() = newPose;
      ecm.SetChanged(actorEntity_,
        ignition::gazebo::components::Pose::typeId,
        ignition::gazebo::ComponentState::OneTimeChange);
    }

    auto *worldPoseComp =
      ecm.Component<ignition::gazebo::components::WorldPose>(actorEntity_);
    if (worldPoseComp)
      worldPoseComp->Data() = newPose;

    auto *animTime =
      ecm.Component<ignition::gazebo::components::AnimationTime>(actorEntity_);
    if (animTime) {
      animTime->Data() +=
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(step / velocity_));
      ecm.SetChanged(actorEntity_,
        ignition::gazebo::components::AnimationTime::typeId,
        ignition::gazebo::ComponentState::OneTimeChange);
    }
  }

private:
  ignition::gazebo::Entity actorEntity_{ignition::gazebo::kNullEntity};
  bool animInitialized_{false};
  bool avoidanceEnabled_{false};

  double initRoll_{0.0};
  double initPitch_{0.0};
  double initYaw_{0.0};

  std::vector<std::string> ignoreNames_;
  double velocity_{1.0};
  double obstacleMargin_{1.5};
  double repulsionGain_{2.0};

  std::vector<ignition::math::Vector3d> targets_;
  std::size_t currentTarget_{0};

  ignition::math::Vector3d currentPos_;
};

}  // namespace dynamic_actor

IGNITION_ADD_PLUGIN(
  dynamic_actor::TrajectoryActorPlugin,
  ignition::gazebo::System,
  dynamic_actor::TrajectoryActorPlugin::ISystemConfigure,
  dynamic_actor::TrajectoryActorPlugin::ISystemPreUpdate)
