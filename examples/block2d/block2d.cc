#include "examples/example_base.h"
#include "utils/find_resource.h"
#include <drake/geometry/proximity_properties.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <gflags/gflags.h>
#include <iostream>

DEFINE_bool(test, false,
            "whether this example is being run in test mode, where we solve a "
            "simpler problem");
DEFINE_string(task, "push", "options are push or gait");
DEFINE_double(mu_ee, 0.8, "EFFECTIVE mu_ee with block");
DEFINE_double(mu_g, 0.1, "EFFECTIVE mu_block with ground");
DEFINE_double(x_limit, 0.4, "x limit for gaiting. only works if task set to gait");
DEFINE_double(stiction_vel, 0.05, "stiction vel for contact formulation");
DEFINE_double(smoothing, 0.001, "smoothing factor for contact formulation");

namespace idto {
namespace examples {
namespace block2d {

using drake::geometry::AddCompliantHydroelasticProperties;
using drake::geometry::AddContactMaterial;
using drake::geometry::Box;
using drake::geometry::ProximityProperties;
using drake::geometry::HalfSpace;
using drake::math::RigidTransformd;
using drake::math::RollPitchYaw;
using drake::multibody::CoulombFriction;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::Vector3d;
using utils::FindIdtoResource;

class Block2dExample : public TrajOptExample {
 public:
  Block2dExample() {
    // Set the camera viewpoint
    const Vector3d camera_pose(1.5, 0.0, 0.5);
    const Vector3d target_pose(0.0, 0.0, 0.0);
    meshcat_->SetCameraPose(camera_pose, target_pose);
  }

  void RunPushExperiment(const std::string options_file,
			 const int state_size,
			 const bool test) const {
    TrajOptExampleParams default_options;
    TrajOptExampleParams options =
        drake::yaml::LoadYamlFile<TrajOptExampleParams>(
            FindIdtoResource(options_file), {}, default_options);

    const double mu_ee = FLAGS_mu_ee;
    const double mu_g  = FLAGS_mu_g ;
    options.friction_list = {mu_g, mu_g, mu_g, mu_g, mu_ee};

    // double mu_ee_urdf = mu_ee / (2-mu_ee);

    const double stiction_vel = FLAGS_stiction_vel;
    options.stiction_velocity = stiction_vel;
    const double smoothing = FLAGS_smoothing;
    options.smoothing_factor = smoothing;
    
    if (test) {
      options.mpc = false;
      options.max_iters = 10;
      options.save_solver_stats_csv = false;
      options.play_target_trajectory = false;
      options.play_initial_guess = false;
      options.play_optimal_trajectory = false;
      options.num_threads = 1;
    }

    if (options.mpc) {
      RunStandaloneMPC(options, state_size);
    } else {
      SolveTrajectoryOptimization(options);
    }
  }
  void RunGaitExperiment(const std::string options_file,
			 const int state_size,
			 const bool test) const {
    TrajOptExampleParams default_options;
    TrajOptExampleParams options =
        drake::yaml::LoadYamlFile<TrajOptExampleParams>(
            FindIdtoResource(options_file), {}, default_options);

    const double x_limit = FLAGS_x_limit;
    const double barrier_weight = x_limit / 10;
    options.q_lower_bound[0] = -x_limit;
    options.q_upper_bound[0] = x_limit;
    options.barrier_weight = barrier_weight;

    const double stiction_vel = FLAGS_stiction_vel;
    options.stiction_velocity = stiction_vel;
    const double smoothing = FLAGS_smoothing;
    options.smoothing_factor = smoothing;
    
    if (test) {
      options.mpc = false;
      options.max_iters = 10;
      options.save_solver_stats_csv = false;
      options.play_target_trajectory = false;
      options.play_initial_guess = false;
      options.play_optimal_trajectory = false;
      options.num_threads = 1;
    }

    if (options.mpc) {
      RunStandaloneMPC(options, state_size);
    } else {
      SolveTrajectoryOptimization(options);
    }
  }

 private:
  void CreatePlantModel(MultibodyPlant<double>* plant) const final {
    // Add a block2d arm without gravity
    std::string robot_file =
        // FindIdtoResource("idto/models/planar_ee_2d.urdf");
      FindIdtoResource("idto/models/planar_ee_2d_limited.urdf");
    ModelInstanceIndex ee_model = Parser(plant).AddModels(robot_file)[0];
    RigidTransformd X_ee_model(RollPitchYaw<double>(0, 0, 0), //M_PI_2),
                           Vector3d(0, 0, 0.0));
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base_link"),
                      X_ee_model);
    plant->set_gravity_enabled(ee_model, false);

    // Add a manipuland with sphere contact
    std::string manipuland_file = FindIdtoResource("idto/models/blocks/block2d_lcs.sdf");
    // std::string manipuland_file =
    //     FindIdtoResource("idto/models/box_15cm_manual_contacts.sdf");
    std::vector<ModelInstanceIndex> block_model = Parser(plant).AddModels(manipuland_file);

    // Add the ground
    const drake::Vector4<double> tan(0.87, 0.7, 0.5, 1.0);
    const drake::Vector4<double> green(0.3, 0.6, 0.4, 1.0);
    RigidTransformd X_ground(Vector3d(0.0, 0.0, -0.5));
    RigidTransformd X_table(Vector3d(0.6, 0.0, -0.499));
    plant->RegisterVisualGeometry(plant->world_body(), X_ground, Box(25, 25, 1),
                                  "ground", green);
    plant->RegisterVisualGeometry(plant->world_body(), X_table,
                                  Box(10, 1.5, 1), "table", tan);
    plant->RegisterCollisionGeometry(plant->world_body(), X_ground,
                                     Box(25, 25, 1), "ground",
                                     CoulombFriction<double>(0.0527, 0.0527));
    // plant->RegisterCollisionGeometry(plant->world_body(),
    // 				     RigidTransformd::Identity(),
    // 				     HalfSpace(),
    // 				     "ground_collision",
    // 				     CoulombFriction<double>(0.5, 0.5));

    std::cout << "Model for controller: " << std::endl;
    std::cout << "EE coll: " << plant->GetCollisionGeometriesForBody(plant->GetBodyByName("ee", ee_model))[0] << std::endl;
    // std::cout << "block coll: " << plant->GetCollisionGeometriesForBody(plant->GetBodyByName("block", block_model[0]))[0] << std::endl;
    // std::cout << "ground coll: " << plant->GetCollisionGeometriesForBody(plant->GetBodyByName("ground")) << std::endl;
    // std::cout << "table coll: " << plant->GetCollisionGeometriesForBody(plant->GetBodyByName("table")) << std::endl;
  }

  void CreatePlantModelForSimulation(
      MultibodyPlant<double>* plant) const final {
    // Use hydroelastic contact, and throw instead of point contact fallback
    // plant->set_contact_model(drake::multibody::ContactModel::kHydroelastic);

    // Add a block2d arm, including gravity, with rigid hydroelastic contact
    std::string robot_file =
        // FindIdtoResource("idto/models/planar_ee_2d.urdf");
        FindIdtoResource("idto/models/planar_ee_2d_limited.urdf");
    ModelInstanceIndex ee_model = Parser(plant).AddModels(robot_file)[0];
    RigidTransformd X_ee_model(RollPitchYaw<double>(0, 0, 0), //M_PI_2),
			       Vector3d(0, 0, 0.0));
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base_link"),
                      X_ee_model);
    plant->set_gravity_enabled(ee_model, true);

    // Add a manipuland with compliant hydroelastic contact
    std::string manipuland_file = FindIdtoResource("idto/models/blocks/block2d.sdf");
    // std::string manipuland_file =
    //     FindIdtoResource("idto/models/box_15cm.sdf");
    std::vector<ModelInstanceIndex> block_model = Parser(plant).AddModels(manipuland_file);

    // Add the ground with compliant hydroelastic contact
    const drake::Vector4<double> tan(0.87, 0.7, 0.5, 1.0);
    const drake::Vector4<double> green(0.3, 0.6, 0.4, 1.0);
    RigidTransformd X_ground(Vector3d(0.0, 0.0, -0.5));
    RigidTransformd X_table(Vector3d(0.6, 0.0, -0.499));
    plant->RegisterVisualGeometry(plant->world_body(), X_ground, Box(25, 25, 1),
                                  "ground", green);
    plant->RegisterVisualGeometry(plant->world_body(), X_table,
                                  Box(10, 1.5, 1), "table", tan);
    // plant->RegisterCollisionGeometry(plant->world_body(), X_ground,
    //                                  Box(25, 25, 1), "ground",
    //                                  CoulombFriction<double>(0.5,0.5));

    ProximityProperties ground_proximity;
    AddContactMaterial(3.0, {}, CoulombFriction<double>(0.0527, 0.0527),
                       &ground_proximity);
    AddCompliantHydroelasticProperties(0.1, 5e7, &ground_proximity);
    plant->RegisterCollisionGeometry(plant->world_body(), X_ground,
                                     Box(25, 25, 1), "ground",
                                     ground_proximity);

  }
};

}  // namespace block2d
}  // namespace examples
}  // namespace idto

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  idto::examples::block2d::Block2dExample example;

  if (FLAGS_task == "push"){
    example.RunPushExperiment("idto/examples/block2d/block2d_push.yaml", 17, FLAGS_test);
  }
  else if (FLAGS_task == "gait"){
    example.RunGaitExperiment("idto/examples/block2d/gait.yaml", 17, FLAGS_test);
  }
  
  // example.RunExample("idto/examples/block2d/block2d.yaml", FLAGS_test);
  // example.RunStandaloneExample("idto/examples/block2d/block2d.yaml", 17, FLAGS_test);
  // example.RunWithExperimentFlags("idto/examples/block2d/block2d.yaml", 17, FLAGS_test);

  return 0;
}
