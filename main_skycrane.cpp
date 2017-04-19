#include <iostream>
#include "quad_vesselcontrol.h"

using namespace std;



int main() {


    cout << "Started." << endl;

    VesselControl Curiosity = VesselControl("Curiosity");

    Curiosity.StartEngines();
    Curiosity.alt1 = Curiosity.alt_stream() + 5;
    Curiosity.vessel.control().set_throttle(1);

    Curiosity.SetTopVector = make_tuple(0,1,0);

    while(abs(Curiosity.alt1 - Curiosity.alt_stream()) > 2){
        Curiosity.Loop();
        }

    Curiosity.vessel.control().toggle_action_group(1);;
    Curiosity.vessel.parts().decouplers()[0].decouple();
    cout << "Skycrane extending" << endl;
    Curiosity.CreateLanderVessel("Curiosity Probe");


    while(Curiosity.lander.situation() != krpc::services::SpaceCenter::VesselSituation::landed){
        Curiosity.Loop();
    }

    Curiosity.vessel.control().toggle_action_group(2);

    Curiosity.alt1 += 1000;
    Curiosity.lat1 += 0.05;

    cout << "Cut cables" << endl;

    while(Curiosity.alt_stream() < Curiosity.alt1 - 950){
        Curiosity.Loop();
    }

    Curiosity.vessel.control().set_throttle(0);



}
