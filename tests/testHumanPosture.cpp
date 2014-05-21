/* A human model reaches to two different locations.
 *
 * */

#include <OpenSim/OpenSim.h>

#include <TaskSpace/Controller.h>
#include <TaskSpace/TaskSet.h>
#include <TaskSpace/ConstantStationTrackingTask.h>
#include <TaskSpace/PriorityLevel.h>
#include <TaskSpace/PriorityLevelSet.h>

#include <iostream>

using namespace OpenSim;
using namespace SimTK;

int main()
{
    // Instantiate objects.
    // --------------------
    Model model("FullBodyModel_Hamner2010.osim");
    model.setUseVisualizer(true);
    
    // Add actuators to the model.
    // ---------------------------
    CoordinateSet& coords = model.updCoordinateSet();
    int num_coordinates = coords.getSize();
    for (int i=0; i < num_coordinates; i++ )
    {
        std::string name = coords.get(i).getName();

        coords.get(i).setDefaultClamped(false);
        coords.get(i).setDefaultLocked(false);

        CoordinateActuator * act = new CoordinateActuator(name);
        act->setName(name);
        model.addForce(act);
    }

    // Add task-space controller.
    // --------------------------
    TaskSpace::Controller * controller = new TaskSpace::Controller();
    // TODO better actuator management.
    controller->setActuators(model.updActuators());
    model.addController(controller);

    // Define the goal position task.
    // ------------------------------
    TaskSpace::ConstantStationTrackingTask * goalPosition =
        new TaskSpace::ConstantStationTrackingTask();
    goalPosition->set_body_name("radius_r");
    goalPosition->set_location_in_body(Vec3(-0.015, -0.23, -0.019));
    goalPosition->set_desired_location(Vec3(1.0, 1.0, 0.0));
    TaskSpace::PriorityLevel * priorityLevel =
        new TaskSpace::PriorityLevel();
    controller->upd_priority_levels().adoptAndAppend(priorityLevel);
    priorityLevel->upd_tasks().adoptAndAppend(goalPosition);

    // Prepare to simulate.
    // --------------------
    State& initState = model.initSystem();
    RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    Manager manager(model, integrator);
    manager.setInitialTime(0.0);
    manager.setFinalTime(5.0);
    model.printDetailedInfo(initState, std::cout);

    // Simulate.
    // ---------
    // TODO std::cout << "Press Enter to start simulation..." << std::endl;
    // TODO getchar();
    manager.integrate(initState);

    // Save data.
    // ----------
    // TODO model.printControlStorage("fourlinks_controls.sto");
    // TODO manager.getStateStorage().print("fourlinks_states.sto");

    model.print("FullBodyModel_Hamner2010_with_controller.osim");
}
