//Copyright (c) Matías Saibene
//Licenced under the MIT Licence

//===============================================================
//                  ORBITER MODULE: KLEINVISION_AIRCAR
//
//AIRCAR.cpp
// Control module for AIRCAR vessel class
//===========================================================

//===============================================================
//      I thank the users who helped me get this thing flying:
//francisdrake
//Thunder Chicken
//Urwumpe
//johnnymanly
//===========================================================

#include <cmath>
#define ORBITER_MODULE
#include "AirCar.h"



// 1. vertical lift component (wings and body)

void VLiftCoeff (VESSEL *v, double aoa, double M, double Re, void *context, double *cl, double *cm, double *cd)
{
	const int nabsc = 9;

	static const double AOA[nabsc] = {-180*RAD,-60*RAD,-30*RAD, -2*RAD, 15*RAD,20*RAD,25*RAD,60*RAD,180*RAD};
	static const double CL[nabsc]  = {       0,      0,   -0.4,      0,    0.7,     1,   0.8,     0,      0};
	static const double CM[nabsc]  = {       0,      0,  0.014, 0.0039, -0.006,-0.008,-0.010,     0,      0};
	/* static const double AOA[nabsc] = {-180*RAD,-60*RAD,-30*RAD, -15*RAD, 0*RAD,15*RAD,30*RAD,60*RAD,180*RAD};
	static const double CL[nabsc]  = {   0,    -0.56,   -0.56,   -0.16,  0.15,  0.46,  0.56,  0.56,  0.00};
	static const double CM[nabsc]  = {    0,    0.00,   0.00,     0.00,  0.00,  0.00,  0.00,  0.00,  0.00}; */

	/* static const double AOA[nabsc] = {-180*RAD,-60*RAD,-30*RAD, -2*RAD, 0*RAD,2*RAD,25*RAD,60*RAD,180*RAD};
	static const double CL[nabsc]  = {    0,    -0.56,   -0,      -0,   0.15, 0.25, 0.56,  0.56,    0.00};
	static const double CM[nabsc]  = {    0,   0.005,   0.004,   0.001, 0.000,-0.001,-0.004, 0.005,  0.00}; */

	int i;
	for (i = 0; i < nabsc-1 && AOA[i+1] < aoa; i++);
	if (i < nabsc - 1) {
		double f = (aoa - AOA[i]) / (AOA[i + 1] - AOA[i]);
		*cl = CL[i] + (CL[i + 1] - CL[i]) * f;  // aoa-dependent lift coefficient
		*cm = CM[i] + (CM[i + 1] - CM[i]) * f;  // aoa-dependent moment coefficient
	}
	else {
		*cl = CL[nabsc - 1];
		*cm = CM[nabsc - 1];
	}
	double saoa = sin(aoa);
	double pd = 0.015 + 0.4*saoa*saoa;  // profile drag
	*cd = pd + oapiGetInducedDrag (*cl, AIRCAR_VLIFT_A, 0.2) + oapiGetWaveDrag (M, 0.75, 1.0, 1.1, 0.04);
	// profile drag + (lift-)induced drag + transonic/supersonic wave (compressibility) drag
}

// 2. horizontal lift component (vertical stabilisers and body)

void HLiftCoeff (VESSEL *v, double beta, double M, double Re, void *context, double *cl, double *cm, double *cd)
{
	int i;
	const int nabsc = 8;
	static const double BETA[nabsc] = {-180*RAD,-135*RAD,-90*RAD,-45*RAD,45*RAD,90*RAD,135*RAD,180*RAD};
	static const double CL[nabsc]   = {       0,    +0.3,      0,   -0.3,  +0.3,     0,   -0.3,      0};
	for (i = 0; i < nabsc-1 && BETA[i+1] < beta; i++);
	if (i < nabsc - 1) {
		*cl = CL[i] + (CL[i + 1] - CL[i]) * (beta - BETA[i]) / (BETA[i + 1] - BETA[i]);
	}
	else {
		*cl = CL[nabsc - 1];
	}
	*cm = 0.0;
	*cd = 0.015 + oapiGetInducedDrag (*cl, AIRCAR_HLIFT_A, 0.6) + oapiGetWaveDrag (M, 0.75, 1.0, 1.1, 0.04);
}


//Constructor
AIRCAR::AIRCAR(OBJHANDLE hVessel, int flightmodel) : VESSEL4(hVessel, flightmodel){

	anim_FoldRotateStow = 0;

	fold_Rotate_Stow_proc = 0.0;

	wings_status = WINGS_DEPLOYED;

	showHelp = true;

	Propeller_status = STOPPED;

	m_pXRSound = nullptr;

	propeller_proc = 0.0;

	wings_proc = 0.0;

	hwing = nullptr;

	mh_AirCar = nullptr;

	mh_AirCar_VC = nullptr;

	uimesh_AirCar = 0;

	uimesh_Cockpit = 0;

	hlaileron = nullptr;

	hraileron = nullptr;

	for(int i = 0; i < 2; i++){
		beacon[i] = {0};
	}

	for(int i = 0; i < 2; i++){
		brakelight[i] = {0};
	}

	l1 = nullptr;

	l2 = nullptr;

	helpmsg1 = nullptr;
	helpmsg2 = nullptr;
	helpmsg3 = nullptr;
	helpmsg4 = nullptr;
	helpmsg5 = nullptr;
	helpmsg6 = nullptr;
	helpmsg7 = nullptr;

	th_main = nullptr;

	DefineAnimations();

	lights_on = false;

	parkingBrakeEnabled = false;

	wheels_rotation = 0.0;
}

//Destructor
AIRCAR::~AIRCAR(){

	//Delete XRSound
	delete m_pXRSound;
	
}

//Overloaded callback functions
//Set the capabilities of the vessel class
void AIRCAR::clbkSetClassCaps(FILEHANDLE cfg){

	mh_AirCar = oapiLoadMeshGlobal(MESH_NAME);
	uimesh_AirCar = AddMesh(mh_AirCar);
	SetMeshVisibilityMode(uimesh_AirCar, MESHVIS_EXTERNAL);

	mh_AirCar_VC = oapiLoadMeshGlobal("KV_AirCar_Cockpit");
	uimesh_Cockpit = AddMesh(mh_AirCar_VC);
	SetMeshVisibilityMode(uimesh_Cockpit, MESHVIS_VC);

    //Physical vessel resources
    SetSize(AIRCAR_SIZE);
    SetEmptyMass(AIRCAR_EMPTYMASS);
    SetMaxWheelbrakeForce(2e5);
	SetCrossSections(AIRCAR_CS);
	SetPMI(AIRCAR_PMI);
	SetRotDrag(AIRCAR_RD);
	SetNosewheelSteering(true);
	SetTouchdownPoints(tdvtx_wheels, wheels);

	//Propellant resources
	PROPELLANT_HANDLE GAS = CreatePropellantResource(AIRCAR_FUELMASS);

	//Main engine
	th_main = CreateThruster((Engine_Location), _V(0, 0, 1), AIRCAR_MAXMAINTH, GAS, AIRCAR_ISP);
	CreateThrusterGroup(&th_main, 1, THGROUP_MAIN);


	//Main wings lift surfaces
	hwing = CreateAirfoil3(LIFT_VERTICAL, _V(0, 1.1359, 0), VLiftCoeff, 0, AIRCAR_VLIFT_C, AIRCAR_VLIFT_S, AIRCAR_VLIFT_A);

	CreateAirfoil3(LIFT_HORIZONTAL, (Elevators_mobile_parts_Location), HLiftCoeff, 0, AIRCAR_HLIFT_C, AIRCAR_HLIFT_S, AIRCAR_HLIFT_A);


	//Control surfaces...

	hlaileron = CreateControlSurface3(AIRCTRL_AILERON, 3.0, 0.35, (Axis_aileron_left_Location), AIRCTRL_AXIS_XNEG,1, anim_raileron);

	hraileron = CreateControlSurface3(AIRCTRL_AILERON, 3.0, 0.35, (Axis_aileron_right_Location), AIRCTRL_AXIS_XPOS,1, anim_laileron);

	CreateControlSurface3(AIRCTRL_ELEVATOR, 3.0*2, 1, (Axis_elevator_Location), AIRCTRL_AXIS_XPOS, 1, anim_elevator);

	CreateControlSurface3(AIRCTRL_ELEVATORTRIM, 3.0*2, 0.5, (Axis_elevator_Location), AIRCTRL_AXIS_XPOS, 1, anim_elevator_trim);

	CreateControlSurface3(AIRCTRL_RUDDER, 3.0, 0.35, (Axis_rudder_left_Location), AIRCTRL_AXIS_YPOS,
	1, anim_left_rudder);

	CreateControlSurface3(AIRCTRL_RUDDER, 3.0, 0.35, (Axis_rudder_right_Location), AIRCTRL_AXIS_YPOS,
	1, anim_right_rudder);

	//Define beacons

	static VECTOR3 beaconpos[2] = {{Beacon1_Location}, {Beacon2_Location}};
	static VECTOR3 beaconcol = {0, 1, 0};

	for(int i = 0; i < 2; i++){
		beacon[i].shape = BEACONSHAPE_STAR;
		beacon[i].pos = beaconpos+i;
		beacon[i].col = &beaconcol;
		beacon[i].size = 0.25;
		beacon[i].falloff = 0.4;
		beacon[i].period = 1;
		beacon[i].duration = 0.1;
		beacon[i].tofs = 0.2;
		beacon[i].active = false;
		AddBeacon(beacon+i);
	}

	//Define brake lights (more beacons)

	static VECTOR3 brakelights[2] = {{Brake_light_1_Location}, {Brake_light_2_Location}};
	static VECTOR3 brakelightscolor = {1, 0, 0};

	for(int i = 0; i < 2; i++){
		brakelight[i].shape = BEACONSHAPE_DIFFUSE;
		brakelight[i].pos = brakelights+i;
		brakelight[i].col = &brakelightscolor;
		brakelight[i].size = 0.25;
		brakelight[i].falloff = 0.4;
		brakelight[i].period = 0;
		brakelight[i].duration = 0.1;
		brakelight[i].tofs = 0.2;
		brakelight[i].active = false;
		AddBeacon(brakelight+i);
	}

	MakeAnnotationFormat();
}


//Define animations
void AIRCAR::DefineAnimations(void){

	////Fold half wing and aileron (left and right) and open wings "doors".

	static unsigned int FoldLeftWingAileronGrp[2] = {Wing_left_aileron_Id, Wing_left_fold_Id};
	static MGROUP_ROTATE FoldLeftWingAileron(
		0,
		FoldLeftWingAileronGrp,
		2,
		(Axis_Left_wing_fold_Location),
		_V(1, 0, 0),
		(float)(180*RAD)
	);

	static unsigned int FoldRightWingAileronGrp[2] = {Wing_right_aileron_Id, Wing_right_fold_Id};
	static MGROUP_ROTATE FoldRightWingAileron(
		0,
		FoldRightWingAileronGrp,
		2,
		(Axis_right_wing_fold_Location),
		_V(1, 0, 0),
		(float)(180*RAD)
	);

	anim_FoldRotateStow = CreateAnimation(0.0);
	AddAnimationComponent(anim_FoldRotateStow, 0, 0.25, &FoldLeftWingAileron);
	AddAnimationComponent(anim_FoldRotateStow, 0, 0.25, &FoldRightWingAileron);

	static unsigned int OpenDoorLeftWingGrp[1] = {Wings_doors_left_Id};
	static MGROUP_ROTATE OpenDoorLeftWing(
		0,
		OpenDoorLeftWingGrp,
		1,
		(Axis_wing_door_left_Location),
		_V(0, 0, 1),
		(float)(-105*RAD)
	);

	static unsigned int OpenDoorRightWingGrp[1] = {Wings_doors_right_Id};
	static MGROUP_ROTATE OpenDoorRightWing(
		0,
		OpenDoorRightWingGrp,
		1,
		(Axis_wing_door_right_Location),
		_V(0, 0, 1),
		(float)(105*RAD)
	);

	AddAnimationComponent(anim_FoldRotateStow, 0, 0.25, &OpenDoorLeftWing);
	AddAnimationComponent(anim_FoldRotateStow, 0, 0.25, &OpenDoorRightWing);

	////Rotate Wings, Ailerons, etc...

	static unsigned int RotateWingLeftGrp[1] = {Wing_left_Id};
	static MGROUP_ROTATE RotateWingLeft(
		0,
		RotateWingLeftGrp,
		1,
		(Axis_Left_wing_Location),
		_V(0, 0, 1),
		(float)(-90*RAD)
	);

	static unsigned int RotateWingRightGrp[1] = {Wing_right_Id};
	static MGROUP_ROTATE RotateWingRight(
		0,
		RotateWingRightGrp,
		1,
		(Axis_Right_wing_Location),
		_V(0, 0, 1),
		(float)(90*RAD)
	);

	AddAnimationComponent(anim_FoldRotateStow, 0.25, 0.50, &RotateWingLeft);
	AddAnimationComponent(anim_FoldRotateStow, 0.25, 0.50, &RotateWingRight);

	static unsigned int RotateWingLeftFoldGrp[1] = {Wing_left_fold_Id};
	static MGROUP_ROTATE RotateWingLeftFold(
		0,
		RotateWingLeftFoldGrp,
		1,
		(Axis_Left_wing_Location),
		_V(0, 0, 1),
		(float)(-90*RAD)
	);

	static unsigned int RotateWingRightFoldGrp[1] = {Wing_right_fold_Id};
	static MGROUP_ROTATE RotateWingRightFold(
		0,
		RotateWingRightFoldGrp,
		1,
		(Axis_Right_wing_Location),
		_V(0, 0, 1),
		(float)(90*RAD)
	);

	AddAnimationComponent(anim_FoldRotateStow, 0.25, 0.50, &RotateWingLeftFold);
	AddAnimationComponent(anim_FoldRotateStow, 0.25, 0.50, &RotateWingRightFold);

	static unsigned int RotateWingLeftAileronGrp[1] = {Wing_left_aileron_Id};
	static MGROUP_ROTATE RotateWingLeftAileron(
		0,
		RotateWingLeftAileronGrp,
		1,
		(Axis_Left_wing_Location),
		_V(0, 0, 1),
		(float)(-90*RAD)
	);

	static unsigned int RotateWingRightAileronGrp[1] = {Wing_right_aileron_Id};
	static MGROUP_ROTATE RotateWingRightAileron(
		0,
		RotateWingRightAileronGrp,
		1,
		(Axis_Right_wing_Location),
		_V(0, 0, 1),
		(float)(90*RAD)
	);

	AddAnimationComponent(anim_FoldRotateStow, 0.25, 0.50, &RotateWingLeftAileron);
	AddAnimationComponent(anim_FoldRotateStow, 0.25, 0.50, &RotateWingRightAileron);

	////Stow things

	static unsigned int StowLeftWingGrp[1] = {Wing_left_Id};
	static MGROUP_ROTATE StowLeftWing(
		0,
		StowLeftWingGrp,
		1,
		(Axis_Left_wing_Location),
		_V(1, 0, 0),
		(float)(-82*RAD)
	);

	static unsigned int StowRightWingGrp[1] = {Wing_right_Id};
	static MGROUP_ROTATE StowRightWing(
		0,
		StowRightWingGrp,
		1,
		(Axis_Right_wing_Location),
		_V(1, 0, 0),
		(float)(-82*RAD)
	);

	AddAnimationComponent(anim_FoldRotateStow, 0.75, 1, &StowLeftWing);
	AddAnimationComponent(anim_FoldRotateStow, 0.75, 1, &StowRightWing);

	static unsigned int StowLeftWingFoldGrp[1] = {Wing_left_fold_Id};
	static MGROUP_ROTATE StowLeftWingFold(
		0,
		StowLeftWingFoldGrp,
		1,
		(Axis_Left_wing_Location),
		_V(1, 0, 0),
		(float)(-90*RAD)
	);

	static unsigned int StowRightWingFoldGrp[1] = {Wing_right_fold_Id};
	static MGROUP_ROTATE StowRightWingFold(
		0,
		StowRightWingFoldGrp,
		1,
		(Axis_Right_wing_Location),
		_V(1, 0, 0),
		(float)(-90*RAD)
	);

	AddAnimationComponent(anim_FoldRotateStow, 0.75, 1, &StowLeftWingFold);
	AddAnimationComponent(anim_FoldRotateStow, 0.75, 1, &StowRightWingFold);

	static unsigned int StowLeftAileronGrp[1] = {Wing_left_aileron_Id};
	static MGROUP_ROTATE StowLeftAileron(
		0,
		StowLeftAileronGrp,
		1,
		(Axis_Left_wing_Location),
		_V(1, 0, 0),
		(float)(-90*RAD)
	);

	static unsigned int StowRightAileronGrp[1] = {Wing_right_aileron_Id};
	static MGROUP_ROTATE StowRightAileron(
		0,
		StowRightAileronGrp,
		1,
		(Axis_Right_wing_Location),
		_V(1, 0, 0),
		(float)(-90*RAD)
	);

	AddAnimationComponent(anim_FoldRotateStow, 0.75, 1, &StowLeftAileron);
	AddAnimationComponent(anim_FoldRotateStow, 0.75, 1, &StowRightAileron);

	static unsigned int CloseDoorLeftWingGrp[1] = {Wings_doors_left_Id};
	static MGROUP_ROTATE CloseDoorLeftWing(
		0,
		CloseDoorLeftWingGrp,
		1,
		(Axis_wing_door_left_Location),
		_V(0, 0, 1),
		(float)(105*RAD)
	);

	static unsigned int CloseDoorRightWingGrp[1] = {Wings_doors_right_Id};
	static MGROUP_ROTATE CloseDoorRightWing(
		0,
		CloseDoorRightWingGrp,
		1,
		(Axis_wing_door_right_Location),
		_V(0, 0, 1),
		(float)(-105*RAD)
	);

	AddAnimationComponent(anim_FoldRotateStow, 0.75, 1, &CloseDoorLeftWing);
	AddAnimationComponent(anim_FoldRotateStow, 0.75, 1, &CloseDoorRightWing);

	static unsigned int StowElevatorsGrp[4] = {Elevators_Id, Elevators_mobile_parts_Id, Elevators_rudder_left_Id, Elevators_rudder_right_Id};
	static MGROUP_TRANSLATE StowElevators(
		0,
		StowElevatorsGrp,
		4,
		_V(0, 0, 0.7)
	);

	AddAnimationComponent(anim_FoldRotateStow, 0.75, 1, &StowElevators);

	//Control surfaces...

	static unsigned int ElevatorsGrp[1] = {Elevators_Id};
	static MGROUP_ROTATE Elevators(
		0,
		ElevatorsGrp,
		1,
		(Axis_elevator_Location),
		_V(1, 0, 0),
		(float)(45*RAD)
	);

	anim_elevator = CreateAnimation(0.5);
	AddAnimationComponent(anim_elevator, 0, 1, &Elevators);

	static unsigned int ElevatorTrimGrp[1] = {Elevators_Id};
	static MGROUP_ROTATE ElevatorTrim(
		0,
		ElevatorTrimGrp,
		1,
		(Axis_elevator_Location),
		_V(1, 0, 0),
		(float)(22.5*RAD)
	);

	anim_elevator_trim = CreateAnimation(0.5);
	AddAnimationComponent(anim_elevator_trim, 0, 1, &ElevatorTrim);

	static unsigned int LRudderGrp[1] = {Elevators_rudder_left_Id};
	static MGROUP_ROTATE LRudder(
		0,
		LRudderGrp,
		1,
		(Axis_rudder_left_Location),
		_V(0, 1, 0),
		(float)(22.5*RAD)
	);

	anim_left_rudder = CreateAnimation(0.5);
	AddAnimationComponent(anim_left_rudder, 0, 1, &LRudder);

	static unsigned int RRudderGrp[1] = {Elevators_rudder_right_Id};
	static MGROUP_ROTATE RRudder(
		0,
		RRudderGrp,
		1,
		(Axis_rudder_right_Location),
		_V(0, 1, 0),
		(float)(22.5*RAD)
	);

	anim_right_rudder = CreateAnimation(0.5);
	AddAnimationComponent(anim_right_rudder, 0, 1, &RRudder);

	static unsigned int LAileronGrp[1] = {Wing_left_aileron_Id};
	static MGROUP_ROTATE LAileron(
		0,
		LAileronGrp,
		1,
		(Axis_aileron_left_Location),
		_V(1, 0, 0),
		(float)(45*RAD)
	);

	anim_laileron = CreateAnimation(0.5);
	AddAnimationComponent(anim_laileron, 0, 1, &LAileron);

	static unsigned int RAileronGrp[1] = {Wing_right_aileron_Id};
	static MGROUP_ROTATE RAileron(
		0,
		RAileronGrp,
		1,
		(Axis_aileron_right_Location),
		_V(1, 0, 0),
		(float)(45*RAD)
	);

	anim_raileron = CreateAnimation(0.5);
	AddAnimationComponent(anim_raileron, 0, 1, &RAileron);

	////Misc animations
	
	static unsigned int RotPropellerGrp[1] = {Propeller_Id};
	static MGROUP_ROTATE RotPropeller(
		0,
		RotPropellerGrp,
		1,
		(Propeller_Location),
		_V(0, 0, 1),
		(float)(360*RAD)
	);
	
	anim_propeller = CreateAnimation(0.0);
	AddAnimationComponent(anim_propeller, 0, 1, &RotPropeller);

	//wheels rotation
	static unsigned int FrontWheelsRotateGrp[1] = {Wheels_front_Id};
	static MGROUP_ROTATE FrontWheelsRotate(
		0,
		FrontWheelsRotateGrp,
		1,
		(Wheels_front_Location),
		_V(1, 0, 0),
		(float)(360*RAD)
	);

	anim_wheels = CreateAnimation(0.0);
	AddAnimationComponent(anim_wheels, 0, 1, &FrontWheelsRotate);

	static unsigned int RearWheelsRotateGrp[1] = {Wheels_rear_Id};
	static MGROUP_ROTATE RearWheelsRotate(
		0,
		RearWheelsRotateGrp,
		1,
		(Wheels_rear_Location),
		_V(1, 0, 0),
		(float)(360*RAD)
	);

	AddAnimationComponent(anim_wheels, 0, 1, &RearWheelsRotate);
}

///////////Load status from scenario file

void AIRCAR::clbkLoadStateEx(FILEHANDLE scn, void *vs){

	char *line;

	while(oapiReadScenario_nextline(scn, line)){
		ParseScenarioLineEx(line, vs);
	}

}

void AIRCAR::clbkSaveState(FILEHANDLE scn){

	char cbuf[256];

	SaveDefaultState(scn);

}

////////////Logic for trigger animations


void AIRCAR::StowWings(void){
	ActivateStowWings((wings_status == WINGS_DEPLOYED || wings_status == WINGS_DEPLOYING) ?
		WINGS_STOWING : WINGS_DEPLOYING);
}

void AIRCAR::ActivateStowWings(WingStatus action){
	wings_status = action;
}

////////////Running animations...

void AIRCAR::clbkPreStep(double simt, double simdt, double mjd){

	//Wheel animation
	VECTOR3 speed = _V(0, 0, 0);
	GetGroundspeedVector(FRAME_LOCAL, speed);

	double rotation_speed = speed.z / (2 * PI * 0.0286);

	wheels_rotation = std::fmod(wheels_rotation + oapiGetSimStep() * rotation_speed, 1.0);

	SetAnimation(anim_wheels, wheels_rotation);

	double alt = GetAltitude();
	double grnspd = GetGroundspeed();


	double pwr = GetThrusterLevel(th_main);
	double prp = GetAnimation(anim_propeller);
	double msimdt = simdt * PROPELLER_ROTATION_SPEED;
	double da = msimdt * 0.1 + (pwr * 0.1);

	propeller_proc = prp + da;

	if(prp < 1){
		SetAnimation(anim_propeller, propeller_proc);
		//SetAnimation(anim_wheels, propeller_proc);
	} else {
		SetAnimation(anim_propeller, 0.0);
		//SetAnimation(anim_wheels, 0.0);
	}

//Thanks johnnymanly
/*pwr = vi:get_thrusterlevel(thmain)
  prp = vi:get_animation(anim_Prop)
  da = simdt * 0.1 + (pwr * 0.1)
  prp_proc = prp + da

  if
    prp < 1
  then
    vi:set_animation(anim_Prop,prp_proc)
  else
    vi:set_animation(anim_Prop)
  end*/
	
	m_pXRSound->PlayWav(engine_idle, true, 1.0);
	
	if(alt > 1500){
		m_pXRSound->PlayWav(engine_far, false, 1.0);
	}

	SetAnnotationHelp();
}

void AIRCAR::clbkPostCreation(){

	m_pXRSound = XRSound::CreateInstance(this);

	m_pXRSound->LoadWav(engine_far, "XRSound\\KleinVision_AirCar\\engine_far.wav", XRSound::PlaybackType::Wind);

	m_pXRSound->LoadWav(XRSound::MainEngines, "XRSound\\KleinVision_AirCar\\engine.wav", XRSound::PlaybackType::Global);
	m_pXRSound->SetDefaultSoundEnabled(XRSound::MainEngines, "XRSound\\KleinVision_AirCar\\engine.wav");

	m_pXRSound->LoadWav(engine_idle, "XRSound\\KleinVision_AirCar\\engine_idle.wav", XRSound::PlaybackType::BothViewClose);

	m_pXRSound->LoadWav(rotate, "XRSound\\Default\\Rotate.wav", XRSound::PlaybackType::Global);

}

void AIRCAR::clbkPostStep(double simt, double simdt, double mjd){

	UpdateStowAnimation(simdt);

}


////////////Functions for animations

void AIRCAR::UpdateStowAnimation(double simdt){
    
	if (wings_status >= WINGS_DEPLOYING) {
        
		double da = simdt * WINGS_OPERATING_SPEED;

        if (wings_status == WINGS_DEPLOYING) {
            if (fold_Rotate_Stow_proc > 0.0) fold_Rotate_Stow_proc = max(0.0, fold_Rotate_Stow_proc - da);
            else wings_status = WINGS_DEPLOYED;
        } else {
            if (fold_Rotate_Stow_proc < 1.0) fold_Rotate_Stow_proc = min(1.0, fold_Rotate_Stow_proc + da);
            else wings_status = WINGS_STOWED;
        }
        SetAnimation(anim_FoldRotateStow, fold_Rotate_Stow_proc);
    }
}

void AIRCAR::ParkingBrake(){

	if(!parkingBrakeEnabled){
		SetWheelbrakeLevel(1, 0 , true);
		parkingBrakeEnabled = true;
	} else {
		SetWheelbrakeLevel(0, 0, true);
        parkingBrakeEnabled = false;
	}
}

void AIRCAR::ActivateBeacons(void){

	for(int i = 0; i < 2; i++){
		if(!beacon[i].active){
				beacon[i].active = true;
		} else {
				beacon[i].active = false;
		}
	}

}

void AIRCAR::ActivateBrakeLights(void){

	for(int i = 0; i < 2; i++){
		if(!brakelight[i].active){
			brakelight[i].active = true;
		} else {
			brakelight[i].active = false;
		}
	}

}

void AIRCAR::LightsControl(void){

	if(!lights_on){
		l1 = AddSpotLight((Light1_Location), _V(0, 0, 1), 1000, 1e-3, 0, 2e-3, 25*RAD, 45*RAD, col_d, col_s, col_a);
		l2 = AddSpotLight((Light2_Location), _V(0, 0, 1), 1000, 1e-3, 0, 2e-3, 25*RAD, 45*RAD, col_d, col_s, col_a);
		lights_on = true;
	} else {
		DelLightEmitter(l1);
		DelLightEmitter(l2);
		lights_on = false;
	}

}

void AIRCAR::SetAnnotationHelp(){

	const char *title = "";
	const char *subtitle = "";

	const char *hlpbeacons = "";
	const char *hlplights = "";
	const char *hlpwings = "";
	const char *hlpsteer = "";

	const char *hlpbrakes = "";

	const char *hlphelp = "";

	if(showHelp == true){
		title = ">>>KleinVision AirCar<<<";
		subtitle = "Key help";

		hlpbeacons = "Press B to activate beacons";
		hlplights = "Press F to activate lights";


		hlpwings = "Press S to stow/deploy wings";
		hlpsteer = "Steer/brake with COMMA , and PERIOD . ";
		hlpbrakes = "Engage parking brake with NUMPAD ENTER";

		hlphelp = "Press K to display/hide this help";

	} else {

		title = "";
		subtitle = "";

		hlpbeacons = "";
		hlplights = "";
		hlpwings = "";
		hlpsteer = "";
		hlpbrakes = "";

		hlphelp = "";

	}

	oapiAnnotationSetText(helpmsg1, const_cast<char *>(title));
	oapiAnnotationSetText(helpmsg2, const_cast<char *>(subtitle));

	oapiAnnotationSetText(helpmsg3, const_cast<char *>(hlpbeacons));
	oapiAnnotationSetText(helpmsg4, const_cast<char *>(hlplights));
	oapiAnnotationSetText(helpmsg5, const_cast<char *>(hlpwings));
	oapiAnnotationSetText(helpmsg6, const_cast<char *>(hlpsteer));
	oapiAnnotationSetText(helpmsg7, const_cast<char *>(hlpbrakes));


	oapiAnnotationSetText(helpmsg8, const_cast<char *>(hlphelp));
}

void AIRCAR::MakeAnnotationFormat(){

	helpmsg1 = oapiCreateAnnotation(true, 1.75, _V(0, 1,0));
	oapiAnnotationSetPos(helpmsg1, 0.3, 0.1, 0.75, 0.12);

	helpmsg2 = oapiCreateAnnotation(true, 1.5, _V(0, 1, 0));
	oapiAnnotationSetPos(helpmsg2, 0.4, 0.15, 0.75, 0.16);



	helpmsg3 = oapiCreateAnnotation(true, 1, _V(0, 1, 0));
	oapiAnnotationSetPos(helpmsg3, 0.3, 0.25, 0.75, 0.30);

	helpmsg4 = oapiCreateAnnotation(true, 1, _V(0, 1, 0));
	oapiAnnotationSetPos(helpmsg4, 0.3, 0.32, 0.75, 0.38);

	helpmsg5 = oapiCreateAnnotation(true, 1, _V(0, 1, 0));
	oapiAnnotationSetPos(helpmsg5, 0.3, 0.40, 0.75, 0.44);

	helpmsg6 = oapiCreateAnnotation(true, 1, _V(0, 1, 0));
	oapiAnnotationSetPos(helpmsg6, 0.3, 0.46, 0.75, 0.50);

	helpmsg7 = oapiCreateAnnotation(true, 1, _V(0, 1, 0));
	oapiAnnotationSetPos(helpmsg7, 0.3, 0.52, 0.75, 0.56);

	helpmsg8 = oapiCreateAnnotation(true, 1, _V(0, 1, 0));
	oapiAnnotationSetPos(helpmsg8, 0.3, 0.58, 0.75, 0.62);


}

int AIRCAR::clbkConsumeDirectKey(char *kstate){

	return 0;
	
}

int AIRCAR::clbkConsumeBufferedKey(DWORD key, bool down, char *kstate){

	if(key == OAPI_KEY_S && down){
		StowWings();
		return 1;
	}
	if(key == OAPI_KEY_B && down){
		ActivateBeacons();
		return 1;
	}
	if(key == OAPI_KEY_F && down){
		LightsControl();
		return 1;
	}
	if(key == OAPI_KEY_NUMPADENTER && down){
        ParkingBrake();
        return 1;
    }

	if(key == OAPI_KEY_K && down){
		if(!showHelp){
			showHelp = true;
		} else {
			showHelp = false;
		}
	}
	return 0;
}

bool AIRCAR::clbkLoadVC(int id){

	static VCMFDSPEC mfds_1 {static_cast<DWORD>(uimesh_Cockpit), MFD1_Id};
	oapiVCRegisterMFD(MFD_LEFT, &mfds_1);

	static VCMFDSPEC mfds_2 {static_cast<DWORD>(uimesh_Cockpit), MFD2_Id};
	oapiVCRegisterMFD(MFD_RIGHT, &mfds_2);


	switch(id){

		case 0:
			SetCameraOffset(VC_camera1_Location);
			SetCameraDefaultDirection(_V(0, 0, 1));
			SetCameraRotationRange(120*RAD, 120*RAD, 60*RAD, 60*RAD);
			oapiVCSetNeighbours(-1, 1, -1, -1);
		break;

		case 1:
			SetCameraOffset(VC_camera2_Location);
			SetCameraDefaultDirection(_V(0, 0, 1));
			SetCameraRotationRange(120*RAD, 120*RAD, 60*RAD, 60*RAD);
			oapiVCSetNeighbours(1, -1, -1, -1);
		break;

	}

	return true;

}

////////////////////////////////////
DLLCLBK void InitModule(HINSTANCE hModule){

}

DLLCLBK void ExitModule(HINSTANCE *hModule){

}

/////////Vessel initialization

DLLCLBK VESSEL *ovcInit(OBJHANDLE hvessel, int flightmodel){

    return new AIRCAR(hvessel, flightmodel);

}

////////////Vessel memory cleanup

DLLCLBK void ovcExit(VESSEL *vessel){

    if(vessel) delete(AIRCAR*)vessel;

}
