#ifndef _DRONE_VESSELCONTROL_H_
#define _DRONE_VESSELCONTROL_H_

#include <iostream>

#include <cmath>

#include <krpc.hpp>
#include <krpc/services/space_center.hpp>
#include <krpc/services/infernal_robotics.hpp>

#include "pid.h"
#include "tuple_operations.h"

using namespace std;


class VesselControl{

    public:
        VesselControl(string name);
        void StartEngines();
        void Loop();
        void CreateLanderVessel(string name);
        void ExtendGear();
        ~VesselControl();
        krpc::services::SpaceCenter::Vessel vessel,lander;

        //STREAMS
            krpc::Stream<tuple<double, double, double>> vel_stream;
            tuple<double, double, double> velvec_surf;
            //stream angular velocity
            krpc::Stream<tuple<double, double, double>> angvel_stream;

            //stream altitude
            krpc::Stream<double> alt_stream;
            float alt0, alt1;

            //stream lat and lon
            krpc::Stream<double> lat_stream;
            krpc::Stream<double> lon_stream;
            double lat0, lat1;
            double lon0, lon1;
            double lonVelOverride = 0;
            double latVelOverride = 0;

        //DEFINE VECTORS
            //define setpoint direction and TopVector in ref_frame_surf
            tuple<double, double, double> SetForeVector = make_tuple(1,0,0);
            tuple<double, double, double> SetTopVector = make_tuple(0,1,0);

        //PID CONTROLLERS
            //lat and lon guidance velocity P controller
            PID LonVelGuidanceVelPID = PID(300,-300,4,0,0);
            PID LatVelGuidanceVelPID = PID(300,-300,4,0,0);

            //lat and lon guidance adjustment P controller
            PID LatGuidanceAdjustPID = PID(0.4,-0.4,0.05,0.01,0);
            PID LonGuidanceAdjustPID = PID(0.4,-0.4,0.05,0.01,0);

            //Rotational velocity control setup
            PID PitchVelControlPID 		= PID(3,	-3,	0.06,	0.035,	0);
            PID YawVelControlPID 		= PID(3,	-3,	0.06,	0.035,	0);
            PID RollVelControlPID 		= PID(1,	-1,	0.005,	0.005,	0);

            //Rotational torque control setup
            PID PitchTorqueControlPID	= PID(0.4,	-0.4,	0.25,	0,		0);
            PID YawTorqueControlPID		= PID(0.4,	-0.4,	0.25,	0,		0);
            PID RollTorqueControlPID	= PID(0.2,	-0.2,	0.,	0,		0);

            //Altitude speed control setup
            PID VertSpeedControlPID		= PID(40,	-40,		0.7,		0,		0);

            //Altitude throttle control setup
            PID ThrottleControlPID		= PID(0.8,	0,		0.15,	0.3,	0);



    private:
        krpc::services::SpaceCenter::Vessel FindVessel(string name);

        double LatSpeedSP, LonSpeedSP;

        double LatAdjust = 0 , LonAdjust = 0;
        double LatSpeedAdjust = 0, LonSpeedAdjust = 0;

        float pitchVelSP = 0, yawVelSP = 0, rollVelSP = 0;
        float midval = 0, pitchAdjust = 0, yawAdjust = 0, rollAdjust = 0;
        float vertVelSP = 0;
        float thrott = 0;

        //define facing vectors in in ref_frame_vessel
        tuple<double, double, double> TopVector = make_tuple(0,0,-1);
        tuple<double, double, double> ForeVector = make_tuple(0,1,0);
        tuple<double, double, double> StarVector = make_tuple(1,0,0);

        //angular velocity vectors converted to vessel reference frame
        tuple<double, double, double> angVel_vessel;

        //Facing vectors converted to surface reference frame
        tuple<double, double, double> TopVector_surface, StarVector_surface, ForeVector_surface, attitudeError;

        //REFERENCE FRAMES
        krpc::services::SpaceCenter::ReferenceFrame ref_frame_surf;
        krpc::services::SpaceCenter::ReferenceFrame ref_frame_orbit_body;
        krpc::services::SpaceCenter::ReferenceFrame ref_frame_nonrot;
        krpc::services::SpaceCenter::ReferenceFrame ref_frame_orb;
        krpc::services::SpaceCenter::ReferenceFrame ref_frame_vessel;

        //Engines
        krpc::services::SpaceCenter::Part
        FR1Engine,FR2Engine,FL1Engine,FL2Engine,BR1Engine,BR2Engine,BL1Engine,BL2Engine;

};




#endif
