// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Demonstration of a steering path-follower PID controller with two alternatives.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================
#include <chrono>

#include "chrono/utils/ChFilters.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_models/vehicle/hmmwv/HMMWV.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_matlab/ChMatlabEngine.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;
using namespace std::chrono;


// =============================================================================
// Select Path Follower, uncomment to select the pure PID steering controller
#define USE_PID 1
// The extended steering controller only works inside the path limits
//#define USE_XT 1
// The simple realistic steering controller should start inside the path limits, after passing the last point
// a) closed loop course: the vehicle goes into the next round
// b) open loop course (this example):  the vehicle keeps the last driving direction (forever)
//#define USE_SR 1
// =============================================================================
// Problem parameters

ChMatrixDynamic<> ResultX(2000, 1);
ChMatrixDynamic<> ResultY(2000, 1);
ChMatrixDynamic<> ResultZ(2000, 1);
ChMatrixDynamic<> ResultX2(2000, 1);
ChMatrixDynamic<> ResultY2(2000, 1);
ChMatrixDynamic<> ResultZ2(2000, 1);

// Contact method type
ChContactMethod contact_method = ChContactMethod::SMC;

// Type of tire model (RIGID, LUGRE, FIALA, PACEJKA, or TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Type of powertrain model (SHAFTS or SIMPLE)
PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;

// Drive type (FWD, RWD, or AWD)
DrivelineTypeWV drive_type = DrivelineTypeWV::RWD;

// Steering type (PITMAN_ARM or PITMAN_ARM_SHAFTS)
// Note: Compliant steering requires higher PID gains.
SteeringTypeWV steering_type = SteeringTypeWV::PITMAN_ARM;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Input file names for the path-follower driver model
////std::string path_file("paths/straight.txt");
////std::string path_file("paths/curve.txt");
////std::string path_file("paths/NATO_double_lane_change.txt");
std::string path_file("paths/ISO_double_lane_change.txt");

// Initial vehicle location and orientation
ChVector<> initLoc(-125, -125, 0.5);
ChQuaternion<> initRot(1, 0, 0, 0);

// Desired vehicle speed (m/s)
double target_speed = 12;

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 300.0;  // size in X direction
double terrainWidth = 300.0;   // size in Y direction

// Point on chassis tracked by the chase camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation step size
double step_size = 2e-3;
double tire_step_size = 1e-3;

// Simulation end time
double t_end = 100;

// Render FPS
double fps = 60;
//double fps = 1;

// Debug logging
bool debug_output = false;
double debug_fps = 10;

// Output directories
const std::string out_dir = GetChronoOutputPath() + "STEERING_CONTROLLER";
const std::string pov_dir = out_dir + "/POVRAY";

// POV-Ray output
bool povray_output = false;

// Vehicle state output (forced to true if povray output enabled)
bool state_output = false;
int filter_window_size = 20;


// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // ------------------------------
    // Create the vehicle and terrain
    // ------------------------------

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(contact_method);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_hmmwv.SetPowertrainType(powertrain_model);
    my_hmmwv.SetDriveType(drive_type);
    my_hmmwv.SetSteeringType(steering_type);
    my_hmmwv.SetTireType(tire_model);
   
    my_hmmwv.SetTireStepSize(tire_step_size);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(chassis_vis_type);
    my_hmmwv.SetSuspensionVisualizationType(suspension_vis_type);
    my_hmmwv.SetSteeringVisualizationType(steering_vis_type);
    my_hmmwv.SetWheelVisualizationType(wheel_vis_type);
    my_hmmwv.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    RigidTerrain terrain(my_hmmwv.GetSystem());

    MaterialInfo minfo;
    minfo.mu = 0.3f;
    minfo.cr = 0.15f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);
    //std::cout << patch_mat->SetFriction();
    //patch_mat->SetFriction(1.5f);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
    patch->SetColor(ChColor(1, 1, 1));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);

    terrain.Initialize();

    // ----------------------
    // Create the Bezier path
    // ----------------------

    // From data file
    auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));

    ChVectorDynamic<> x(25);
    ChVectorDynamic<> y(25);
    double delta = 1.0 / 25;

    for (int i = 0; i < 25; i++) {
        ChVector<> pos = path->eval(delta * i);
        x(i) = pos.x();
        y(i) = pos.y();
    }

    // Parameterized ISO double lane change (to left)
    //auto path = DoubleLaneChangePath(ChVector<>(-125, -125, 0.1), 13.5, 4.0, 11.0, 50.0, true);
    //plot(path, 100, "plotted_results", false);

    // Parameterized NATO double lane change (to right)
    ////auto path = DoubleLaneChangePath(ChVector<>(-125, -125, 0.1), 28.93, 3.6105, 25.0, 50.0, false);

    ////path->write("my_path.txt");

    // ------------------------
    // Create the driver system
    // ------------------------

#ifdef USE_PID
    ChPathFollowerDriver driver(my_hmmwv.GetVehicle(), path, "my_path", target_speed);
    driver.SetColor(ChColor(0.0f, 0.0f, 0.8f));
    driver.GetSteeringController().SetLookAheadDistance(5);
    driver.GetSteeringController().SetGains(0.8, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();
#endif
#ifdef USE_XT
    ChPathFollowerDriverXT driver(my_hmResultX 0.8f);
    driver.GetSteeringController().SetLookAheadDistance(5);
    driver.GetSteeringController().SetGains(0.4, 1, 1, 1);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();
#endif
#ifdef USE_SR
    const double axle_space = 3.2;
    const bool path_is_closed = false;
    ChPathFollowerDriverSR driver(my_hmmwv.GetVehicle(), path, "my_path", target_speed, path_is_closed,
                                  my_hmmwv.GetVehicle().GetMaxSteeringAngle(), axle_space);
    // driver.GetSteeringController().SetLookAheadDistance(5);
    driver.SetColor(ChColor(0.0f, 0.0f, 0.8f));
    driver.GetSteeringController().SetPreviewTime(0.7);
    driver.GetSteeringController().SetGains(0.1, 5);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();
#endif

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->AttachVehicle(&my_hmmwv.GetVehicle());

#ifdef USE_PID
    vis->SetWindowTitle("Steering PID Controller Demo");
#endif
#ifdef USE_XT
    vis->SetWindowTitle("Steering XT Controller Demo");
#endif
#ifdef USE_SR
    vis->SetWindowTitle("Steering SR Controller Demo");
#endif
    vis->SetHUDLocation(500, 20);
    vis->SetChaseCamera(trackPoint, 6.0, 0.5);
    vis->Initialize();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AddLight(ChVector<>(-150, -150, 200), 300, ChColor(0.7f, 0.7f, 0.7f));
    vis->AddLight(ChVector<>(-150, +150, 200), 300, ChColor(0.7f, 0.7f, 0.7f));
    vis->AddLight(ChVector<>(+150, -150, 200), 300, ChColor(0.7f, 0.7f, 0.7f));
    vis->AddLight(ChVector<>(+150, +150, 200), 300, ChColor(0.7f, 0.7f, 0.7f));

    // Visualization of controller points (sentinel & target)
    irr::scene::IMeshSceneNode* ballS = vis->GetSceneManager()->addSphereSceneNode(0.1f);
    irr::scene::IMeshSceneNode* ballT = vis->GetSceneManager()->addSphereSceneNode(0.1f);
    ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
    ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

    // -----------------
    // Initialize output
    // -----------------

    state_output = state_output || povray_output;

    if (state_output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        driver.ExportPathPovray(out_dir);
    }

    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    utils::ChRunningAverage fwd_acc_GC_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_GC_filter(filter_window_size);

    utils::ChRunningAverage fwd_acc_driver_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_driver_filter(filter_window_size);

    // ---------------
    // Simulation loop
    // ---------------

    // Driver location in vehicle local frame
    ChVector<> driver_pos = my_hmmwv.GetChassis()->GetLocalDriverCoordsys().pos;
   
    // Number of simulation steps between miscellaneous events
    double render_step_size = 1 / fps;
    int render_steps = (int)std::ceil(render_step_size / step_size);
    double debug_step_size = 1 / debug_fps;
    int debug_steps = (int)std::ceil(debug_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    int sim_frame = 0;
    int render_frame = 0;
    int i = 0;
    int j = 0;

    my_hmmwv.GetVehicle().EnableRealtime(true);

    auto start = high_resolution_clock::now();
    while (vis->Run()) {
        // Extract system state
        double time = my_hmmwv.GetSystem()->GetChTime();
        ChVector<> acc_CG = my_hmmwv.GetVehicle().GetChassisBody()->GetPos_dtdt();
        ChVector<> acc_driver = my_hmmwv.GetVehicle().GetPointAcceleration(driver_pos);
        double fwd_acc_CG = fwd_acc_GC_filter.Add(acc_CG.x());
        double lat_acc_CG = lat_acc_GC_filter.Add(acc_CG.y());
        double fwd_acc_driver = fwd_acc_driver_filter.Add(acc_driver.x());
        double lat_acc_driver = lat_acc_driver_filter.Add(acc_driver.y());

        // End simulation
        //if (time >= t_end)
          // break;
        

        if (j == 800)
           break;
        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();


        // Update sentinel and target location markers for the path-follower controller.
        const ChVector<>& pS = driver.GetSteeringController().GetSentinelLocation();
        const ChVector<>& pT = driver.GetSteeringController().GetTargetLocation();
        ballS->setPosition(irr::core::vector3df((irr::f32)pS.x(), (irr::f32)pS.y(), (irr::f32)pS.z()));
        ballT->setPosition(irr::core::vector3df((irr::f32)pT.x(), (irr::f32)pT.y(), (irr::f32)pT.z()));
        

    
        auto Result_pos = my_hmmwv.GetChassis()->GetMarkers()[0]->GetAbsCoord().pos;;
        //std::cout <<  << std::endl;

        if((i % 100)==0 and j<800){
            
            ResultX(j)= Result_pos.x();
            ResultY(j)= Result_pos.y();
            ResultZ(j)= Result_pos.z();
            j+=1;
        }
        i+=1;
        
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Output POV-Ray data
        if (sim_frame % render_steps == 0) {
            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(my_hmmwv.GetSystem(), filename);
            }

            if (state_output) {
                csv << time << driver_inputs.m_steering << driver_inputs.m_throttle << driver_inputs.m_braking;
                csv << my_hmmwv.GetVehicle().GetSpeed();
                csv << acc_CG.x() << fwd_acc_CG << acc_CG.y() << lat_acc_CG;
                csv << acc_driver.x() << fwd_acc_driver << acc_driver.y() << lat_acc_driver;
                csv << std::endl;
            }

            render_frame++;
        }

        // Debug logging
        if (debug_output && sim_frame % debug_steps == 0) {
            GetLog() << "driver acceleration:  " << acc_driver.x() << "  " << acc_driver.y() << "  " << acc_driver.z()
                     << "\n";
            GetLog() << "CG acceleration:      " << acc_CG.x() << "  " << acc_CG.y() << "  " << acc_CG.z() << "\n";
            GetLog() << "\n";
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize("Double lane change", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_hmmwv.Advance(step_size);
        vis->Advance(step_size);

        // Increment simulation frame number
        sim_frame++;
    }


    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << duration.count() << std:: endl;
 
    try{
        GetLog() << "PERFORM TESTS OF MATLAB<->CHRONO INTERACTION\n\n";
        GetLog() << "(please wait few seconds: Matlab engine must be loaded)\n\n";

        // This is the object that you can use to access the Matlab engine.
        // As soon as created, it loads the Matlab engine (if troubles happen, it
        // throws exception).

        ChMatlabEngine matlab_engine;

        //
        // PLOTTING : Trajectory followed by vehicle
        //

        GetLog() << "- Execute plotting command from Chrono...\n\n";

        //
        // pass a Chrono matrix/vector to Matlab
        //

       
        //GetLog() << "Something done ! ... \n\n";
        matlab_engine.PutVariable(ResultX, "ResultX");
        matlab_engine.PutVariable(ResultY, "ResultY");
        matlab_engine.PutVariable(ResultZ, "ResultZ");
        matlab_engine.PutVariable(x, "x");
        matlab_engine.PutVariable(y, "y");

        /*
        matlab_engine.PutVariable(ResultX2, "ResultX2");
        matlab_engine.PutVariable(ResultY2, "ResultY2");
        matlab_engine.PutVariable(ResultZ2, "ResultZ2");
        */
       
        matlab_engine.Eval("figure;f = gcf; grid on; axis equal; plot(x, y, 'r');");
        matlab_engine.Eval(" hold on; plot(ResultX, ResultY, 'go');");

        //matlab_engine.Eval("hold on; plot(ResultX2, ResultY2, 'g');");


        matlab_engine.Eval("legend ('Ground truth trajectory', 'Actual trajectory followed');");
        matlab_engine.Eval("xlabel ('Position X [m]');");
        matlab_engine.Eval("ylabel ('Position Y [m]');");
        matlab_engine.Eval("title ('Trajectory Followed by vehicle');");
        matlab_engine.Eval("exportgraphics(f, 'Trajectory_final_0.3.png', 'Resolution',300);");

         
       
        //GetLog() << "Past Evaluations done ! ... \n\n";

        // Wait some seconds before closing all

        matlab_engine.Eval("pause(60)");

        //GetLog() << "Everything done ! ... \n\n";

    }catch (...) {
        GetLog() << "NOTHING done ! ... \n\n";  // Print error on console, if Matlab did not start.
    }
    
    
    if (state_output)
        csv.write_to_file(out_dir + "/state.out");
    return 0;
}
