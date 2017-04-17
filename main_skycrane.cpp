#include <iostream>
#include "quad_vesselcontrol.h"

using namespace std;



int main() {


    cout << "Started." << endl;

    VesselControl Vessel1 = VesselControl("Drone");


    cout << "Assigned vessel:    " << Vessel1.vessel.name() << endl;

    Vessel1.startEngines();
    Vessel1.alt1 = 1000;
    Vessel1.vessel.control().set_throttle(1);
    Vessel1.retractGear();

    Vessel1.SetTopVector = make_tuple(0,0,1);

    while (true){
        Vessel1.loop() ;
        }
}
