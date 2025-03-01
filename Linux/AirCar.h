#ifndef __AIRCAR_H
#define __AIRCAR_H

#define STRICT 1
#include "OrbiterAPI.h"
#include "VesselAPI.h"
#include "Orbitersdk.h"
#include "KleinVision_AirCar.h"
#include "XRSound.h"

const double WINGS_OPERATING_SPEED = 0.12;

const double AIRCAR_SIZE = 3.5; //Mean radius in meters.

const double AIRCAR_EMPTYMASS = 1100; //Empty mass in kg.

const double AIRCAR_FUELMASS = 100; //Fuel mass in kg.

const double AIRCAR_ISP = 75e3; //Fuel-specific impulse in m/s.

const double AIRCAR_MAXMAINTH = 8e3;

const VECTOR3 AIRCAR_CS = {6.43, 19.44, 3.69};

const VECTOR3 AIRCAR_PMI = {2.33, 3.08, 1.53};

const VECTOR3 AIRCAR_RD = {1, 1, 1}; //Rotation drag coefficients.

//const double AIRCAR_VLIFT_C = 4.8; //Chord lenght in meters.
const double AIRCAR_VLIFT_C = 1.2; //Chord lenght in meters.

const double AIRCAR_VLIFT_S = 36; //Wing area in m^2.
//const double AIRCAR_VLIFT_S = 6.1407; //Wing area in m^2.

const double AIRCAR_VLIFT_A = 1.778; //Wing aspect ratio.
//const double AIRCAR_VLIFT_A = 4.26; //Wing aspect ratio.

const double AIRCAR_HLIFT_C = 0.514; //Chord lenght in meters.

const double AIRCAR_HLIFT_S = 0.3171; //Wing area in m^2.

const double AIRCAR_HLIFT_A = 0.0155; //Wing aspect ratio?.

const double MECHANISM_OPERATING_SPEED = 0.25;

const double PROPELLER_ROTATION_SPEED = 10;

static const int wheels = 14;
static TOUCHDOWNVTX tdvtx_wheels[wheels] = {
    {(TDP10_Location), 39240, 13724, 1.6, 0.1},
    {(TDP3_Location), 39240, 13724, 3.0, 0.2},
    {(TDP4_Location), 39240, 13724, 3.0, 0.2},
    {(TDP1_Location), 2e4, 1e3, 3.0},
    {(TDP5_Location), 2e4, 1e3, 3.0},
    {(TDP6_Location), 2e4, 1e3, 3.0},
    {(TDP7_Location), 2e4, 1e3, 3.0},
    {(TDP8_Location), 2e4, 1e3, 3.0},
    {(TDP9_Location), 2e4, 1e3, 3.0},
    {(TDP_WING1_Location), 2e4, 1e3, 3.0},
    {(TDP_WING2_Location), 2e4, 1e3, 3.0},
    {(TDP_WING3_Location), 2e4, 1e3, 3.0},
    {(TDP_WING4_Location), 2e4, 1e3, 3.0}
};

//AIRCAR class interface
class AIRCAR: public VESSEL4{
    public:

        enum MySounds {engine_idle, engine, engine_far, rotate};

        enum WingStatus {WINGS_DEPLOYED, WINGS_STOWED, WINGS_DEPLOYING, WINGS_STOWING} wings_status;

        enum PropellerStatus {STOPPED, RUNNING, STOPPING, STARTING} Propeller_status;

        AIRCAR(OBJHANDLE hVessel, int flightmodel);
        virtual ~AIRCAR();

        void DefineAnimations(void);
        void UpdateStowAnimation(double);
        void UpdatePropellerAnimation(double);

        void StowWings(void);
        void ActivateStowWings(WingStatus action);
        void ActivatePropeller(PropellerStatus action);
        void runPropeller(void);
        void ActivateBeacons(void);
        void ActivateBrakeLights(void);
        void LightsControl(void);

        void clbkSetClassCaps(FILEHANDLE cfg) override;
        void clbkLoadStateEx(FILEHANDLE scn, void *vs) override;
        void clbkSaveState(FILEHANDLE scn) override;
        void clbkPostStep(double, double, double) override;
        void clbkPreStep(double, double, double) override;
        int clbkConsumeDirectKey(char *kstate) override;
        int clbkConsumeBufferedKey(int, bool, char *) override;
        void clbkPostCreation(void) override;
        //bool clbkLoadVC(int id) override;

        void SetAnnotationHelp();
        void MakeAnnotationFormat();
    
        XRSound *m_pXRSound;

    private:

        //Variable for lights control
        bool lights_on;
        
        unsigned int anim_left_rudder;
        unsigned int anim_right_rudder;
        unsigned int anim_elevator;
        unsigned int anim_elevator_trim;
        unsigned int anim_laileron;
        unsigned int anim_raileron;
        
        unsigned int anim_FoldRotateStow;
        unsigned int anim_propeller;
        unsigned int anim_wheels;
        
        double fold_Rotate_Stow_proc;
        double propeller_proc;

        double wings_proc;
        bool showHelp;
        
        AIRFOILHANDLE hwing;
        MESHHANDLE mh_AirCar, mh_AirCar_VC;
        unsigned int uimesh_AirCar;
        unsigned int uimesh_Cockpit;
        CTRLSURFHANDLE hlaileron, hraileron;
        BEACONLIGHTSPEC beacon[2], brakelight[2];
        LightEmitter *l1, *l2;
        THRUSTER_HANDLE th_main;
        COLOUR4 col_d = {0.9,0.8,1,0};
	    COLOUR4 col_s = {1.9,0.8,1,0};
	    COLOUR4 col_a = {0,0,0,0};
	    COLOUR4 col_white = {1,1,1,0};
        NOTEHANDLE helpmsg1, helpmsg2, helpmsg3, helpmsg4, helpmsg5, helpmsg6;

};


#endif //!__AIRCAR_H