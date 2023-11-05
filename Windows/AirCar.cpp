//Copyright (c) Matías Saibene
//Licenced under the MIT Licence

//===============================================================
//                  ORBITER MODULE: KLEINVISION_AIRCAR
//
//AIRCAR.cpp
// Control module for AIRCAR vessel class
//===========================================================

#define ORBITER_MODULE
#include <cstring>
#include <cstdint>
#include "AirCar.h"
#include <algorithm>
#include <cstdio>

/*
// 1. vertical lift component (code from DeltaGlider)

void VLiftCoeff (VESSEL *v, double aoa, double M, double Re, void *context, double *cl, double *cm, double *cd)
{
	const int nabsc = 9;
	static const double AOA[nabsc] = {-180*RAD,-60*RAD,-30*RAD, -2*RAD, 15*RAD,20*RAD,25*RAD,60*RAD,180*RAD};
	static const double CL[nabsc]  = {       0,      0,   -0.4,      0,    0.7,     1,   0.8,     0,      0};
	static const double CM[nabsc]  = {       0,      0,  0.014, 0.0039, -0.006,-0.008,-0.010,     0,      0};
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
	*cd = pd + oapiGetInducedDrag (*cl, 1.5, 0.7) + oapiGetWaveDrag (M, 0.75, 1.0, 1.1, 0.04);
	// profile drag + (lift-)induced drag + transonic/supersonic wave (compressibility) drag
}

// 2. horizontal lift component (code from DeltaGlider)

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
	*cd = 0.015 + oapiGetInducedDrag (*cl, 1.5, 0.6) + oapiGetWaveDrag (M, 0.75, 1.0, 1.1, 0.04);
}
*/

//Constructor
AIRCAR::AIRCAR(OBJHANDLE hVessel, int flightmodel) : VESSEL4(hVessel, flightmodel){
    
    DefineAnimations();

	fold_aileron_proc = 0.0;

	rotate_left_wing_proc = 0.0;

	stow_left_wing_proc = 0.0;

	WingRotation_status = WR_DEPLOYED;

	fold_status = FW_DEPLOYED;

	WingStow_status = WS_DEPLOYED;

}

//Destructor
AIRCAR::~AIRCAR(){

}

//Overloaded callback functions
//Set the capabilities of the vessel class
void AIRCAR::clbkSetClassCaps(FILEHANDLE cfg){

	THRUSTER_HANDLE th_main;

    //Physical vessel resources
    SetSize(AIRCAR_SIZE);
    SetEmptyMass(AIRCAR_EMPTYMASS);
    SetMaxWheelbrakeForce(2e5);
	SetCrossSections(AIRCAR_CS);
	SetPMI(AIRCAR_PMI);
	SetTouchdownPoints(tdvtx_wheels, wheels);

	//Propellant resources
	PROPELLANT_HANDLE GAS = CreatePropellantResource(AIRCAR_FUELMASS);

	//Main engine
	th_main = CreateThruster((Propeller_Location), _V(0, 0, 1), AIRCAR_MAXMAINTH, GAS, AIRCAR_ISP);
	CreateThrusterGroup(&th_main, 1, THGROUP_MAIN);


	//Control surfaces...
	CreateControlSurface3(AIRCTRL_ELEVATOR, 0.8008, 1.7, Axis_elevator_Location, AIRCTRL_AXIS_AUTO, 1, anim_elevator);

	CreateControlSurface3(AIRCTRL_ELEVATORTRIM, 0.8008, 1.7, Axis_elevator_Location, AIRCTRL_AXIS_AUTO, 1, anim_elevator_trim);

	CreateControlSurface3(AIRCTRL_RUDDER, 0.0924, 0.85, Axis_rudder_left_Location, AIRCTRL_AXIS_AUTO,
	1, anim_left_rudder);

	CreateControlSurface3(AIRCTRL_RUDDER, 0.0924, 0.85, Axis_rudder_right_Location, AIRCTRL_AXIS_AUTO,
	1, anim_right_rudder);

	CreateControlSurface3(AIRCTRL_AILERON, 0.0911, 1.7, Axis_aileron_left_Location, AIRCTRL_AXIS_AUTO,1, anim_raileron);

	CreateControlSurface3(AIRCTRL_AILERON, 0.0911, 1.7, Axis_aileron_right_Location, AIRCTRL_AXIS_AUTO,1, anim_laileron);

	//Add mesh
	AddMesh("KleinVision_AirCar");
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

	anim_FoldAileronLeftWing = CreateAnimation(0.0);
	AddAnimationComponent(anim_FoldAileronLeftWing, 0, 1, &FoldLeftWingAileron);
	AddAnimationComponent(anim_FoldAileronLeftWing, 0, 1, &FoldRightWingAileron);

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

	AddAnimationComponent(anim_FoldAileronLeftWing, 0, 1, &OpenDoorLeftWing);
	AddAnimationComponent(anim_FoldAileronLeftWing, 0, 1, &OpenDoorRightWing);


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

	anim_RotateLeftWing = CreateAnimation(0.0);
	AddAnimationComponent(anim_RotateLeftWing, 0, 1, &RotateWingLeft);
	AddAnimationComponent(anim_RotateLeftWing, 0, 1, &RotateWingRight);

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

	AddAnimationComponent(anim_RotateLeftWing, 0, 1, &RotateWingLeftFold);
	AddAnimationComponent(anim_RotateLeftWing, 0, 1, &RotateWingRightFold);

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

	AddAnimationComponent(anim_RotateLeftWing, 0, 1, &RotateWingLeftAileron);
	AddAnimationComponent(anim_RotateLeftWing, 0, 1, &RotateWingRightAileron);


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

	anim_left_wing_stow = CreateAnimation(0.0);
	AddAnimationComponent(anim_left_wing_stow, 0, 1, &StowLeftWing);
	AddAnimationComponent(anim_left_wing_stow, 0, 1, &StowRightWing);


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

	AddAnimationComponent(anim_left_wing_stow, 0, 1, &StowLeftWingFold);
	AddAnimationComponent(anim_left_wing_stow, 0, 1, &StowRightWingFold);


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

	AddAnimationComponent(anim_left_wing_stow, 0, 1, &StowLeftAileron);
	AddAnimationComponent(anim_left_wing_stow, 0, 1, &StowRightAileron);

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


	AddAnimationComponent(anim_left_wing_stow, 0, 1, &CloseDoorLeftWing);
	AddAnimationComponent(anim_left_wing_stow, 0, 1, &CloseDoorRightWing);

	static unsigned int StowElevatorsGrp[4] = {Elevators_Id, Elevators_mobile_parts_Id, Elevators_rudder_left_Id, Elevators_rudder_right_Id};
	static MGROUP_TRANSLATE StowElevators(
		0,
		StowElevatorsGrp,
		4,
		_V(0, 0, 0.7) //Ajustar el valor de desplazamiento Z aquí.
	);
	
	AddAnimationComponent(anim_left_wing_stow, 0, 1, &StowElevators);


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
}


////////////Logic for trigger animations

void AIRCAR::FoldWing(void){
	ActivateFold((fold_status == FW_DEPLOYED || fold_status == FW_DEPLOYING) ?
		FW_STOWING : FW_DEPLOYING);
}

void AIRCAR::ActivateFold(FoldWingStatus action){

	fold_status = action;

}

void AIRCAR::RotateWings(void){
	ActivateWingRotation((WingRotation_status == WR_DEPLOYED || WingRotation_status == WR_DEPLOYING) ?
		WR_STOWING : WR_DEPLOYING);
}

void AIRCAR::ActivateWingRotation(WingRotationStatus action){
	WingRotation_status = action;
}

void AIRCAR::StowWing(void){
	ActivateStowWing((WingStow_status == WS_DEPLOYED || WingStow_status == WS_DEPLOYING) ?
		WS_STOWING : WS_DEPLOYING);
}

void AIRCAR::ActivateStowWing(WingStowStatus action){
	WingStow_status = action;
}

////////////Running animations...

void AIRCAR::clbkPostStep(double simt, double simdt, double mjd){

	UpdateFoldAnimation(simdt);
	UpdateRotationAnimation(simdt);
	UpdateStowAnimation(simdt);

}

////////////Functions for animations

void AIRCAR::UpdateFoldAnimation(double simdt){
    
	if (fold_status >= FW_DEPLOYING) {
        
		double da = simdt * WINGS_OPERATING_SPEED;

        if (fold_status == FW_DEPLOYING) {
            if (fold_aileron_proc > 0.0) fold_aileron_proc = std::max(0.0, fold_aileron_proc - da);
            else fold_status = FW_DEPLOYED;
        } else {
            if (fold_aileron_proc < 1.0) fold_aileron_proc = std::min(1.0, fold_aileron_proc + da);
            else fold_status = FW_STOWED;
        }
        SetAnimation(anim_FoldAileronLeftWing, fold_aileron_proc);
    }
}

void AIRCAR::UpdateRotationAnimation(double simdt){
	if (WingRotation_status >= WR_DEPLOYING) {
        
		double da = simdt * WINGS_OPERATING_SPEED;

        if (WingRotation_status == WR_DEPLOYING) {
            if (rotate_left_wing_proc > 0.0) rotate_left_wing_proc = std::max(0.0, rotate_left_wing_proc - da);
            else WingRotation_status = WR_DEPLOYED;
        } else {
            if (rotate_left_wing_proc < 1.0) rotate_left_wing_proc = std::min(1.0, rotate_left_wing_proc + da);
            else WingRotation_status = WR_STOWED;
        }
        SetAnimation(anim_RotateLeftWing, rotate_left_wing_proc);
    }
}

void AIRCAR::UpdateStowAnimation(double simdt){
	if (WingStow_status >= WS_DEPLOYING) {
        
		double da = simdt * WINGS_OPERATING_SPEED;

        if (WingStow_status == WS_DEPLOYING) {
            if (stow_left_wing_proc > 0.0) stow_left_wing_proc = std::max(0.0, stow_left_wing_proc - da);
            else WingStow_status = WS_DEPLOYED;
        } else {
            if (stow_left_wing_proc < 1.0) stow_left_wing_proc = std::min(1.0, stow_left_wing_proc + da);
            else WingStow_status = WS_STOWED;
        }
        SetAnimation(anim_left_wing_stow, stow_left_wing_proc);
    }
}


int AIRCAR::clbkConsumeBufferedKey(int key, bool down, char *kstate){

	if(key == OAPI_KEY_1 && down){
		FoldWing();
		return 1;
	}
	if(key == OAPI_KEY_2 && down){
		RotateWings();
		return 1;
	}
	if(key == OAPI_KEY_3 && down){
		StowWing();
		return 1;
	}
	return 0;
}

////////////////////////////////////
DLLCLBK void InitModule(MODULEHANDLE hModule){

}

DLLCLBK void ExitModule(MODULEHANDLE *hModule){

}

/////////Vessel initialization

DLLCLBK VESSEL *ovcInit(OBJHANDLE hvessel, int flightmodel){

    return new AIRCAR(hvessel, flightmodel);

}

////////////Vessel memory cleanup

DLLCLBK void ovcExit(VESSEL *vessel){

    if(vessel) delete(AIRCAR*)vessel;

}