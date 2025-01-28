// Auto generated code file.  Blender: 3.3.9  Blender Tools: (2, 1, 3)
// Date: Tue Jan 28 16:05:05 2025


#include "Orbitersdk.h"
#include <cstdint>

#ifndef __KleinVision_AirCar_H
#define __KleinVision_AirCar_H


    const uint32_t TXIDX_Main_body_dds = 1;
    const uint32_t TXIDX_Chrome_material_dds = 2;
    const uint32_t TXIDX_Wheels_front_dds = 3;
    const uint32_t TXIDX_Wing_left_dds = 4;
    const uint32_t TXIDX_Headlights_dds = 5;
    constexpr auto MESH_NAME = "KleinVision_AirCar";

    const UINT Propeller_shaft_Id = 0;
    const UINT Propeller_Id = 1;
    const UINT Engine_vent_Id = 2;
    const UINT Elevators_mobile_parts_Id = 3;
    const UINT Main_body_Id = 4;
    const UINT Elevators_rudder_left_Id = 5;
    const UINT Wheels_front_Id = 6;
    const UINT Elevators_Id = 7;
    const UINT Elevators_rudder_right_Id = 8;
    const UINT Wing_left_Id = 9;
    const UINT Wing_left_aileron_Id = 10;
    const UINT Wing_left_fold_Id = 11;
    const UINT Wing_right_fold_Id = 12;
    const UINT Wing_right_aileron_Id = 13;
    const UINT Cockpit_Id = 14;
    const UINT Wings_doors_left_Id = 15;
    const UINT Wings_doors_right_Id = 16;
    const UINT Headlights_Id = 17;
    const UINT Wing_right_Id = 18;
    const UINT Wheels_rear_Id = 19;
    const UINT Window_Id = 20;
    constexpr VECTOR3 Propeller_shaft_Location =     {0.0000, 0.8415, -0.8815};
    constexpr VECTOR3 Propeller_Location =     {-0.0000, 1.1359, -2.2926};
    constexpr VECTOR3 Engine_vent_Location =     {0.1383, 1.3013, 0.8849};
    constexpr VECTOR3 Elevators_mobile_parts_Location =     {-0.0000, 0.8981, -3.1068};
    constexpr VECTOR3 Main_body_Location =     {-0.0000, 0.3960, 0.4073};
    constexpr VECTOR3 Window_Location =     {-0.0056, 0.9995, 1.5046};
    constexpr VECTOR3 Elevators_rudder_left_Location =     {-1.0006, 0.7885, -3.4090};
    constexpr VECTOR3 Wheels_front_Location =     {-0.0001, 0.1623, 2.0759};
    constexpr VECTOR3 Elevators_Location =     {-0.0000, 1.1552, -3.6406};
    constexpr VECTOR3 Elevators_rudder_right_Location =     {0.9898, 0.7885, -3.4090};
    constexpr VECTOR3 Wing_left_Location =     {-2.3139, 0.5000, 0.3017};
    constexpr VECTOR3 Wing_left_aileron_Location =     {-3.6388, 0.5013, -0.2995};
    constexpr VECTOR3 Wing_left_fold_Location =     {-2.3950, 0.4997, -0.0418};
    constexpr VECTOR3 Wing_right_fold_Location =     {2.3950, 0.4997, -0.0418};
    constexpr VECTOR3 Wing_right_aileron_Location =     {3.6387, 0.5013, -0.2995};
    constexpr VECTOR3 Cockpit_Location =     {0.0118, 0.7253, 1.3901};
    constexpr VECTOR3 Wings_doors_left_Location =     {-0.7601, 0.9632, 0.4206};
    constexpr VECTOR3 Wings_doors_right_Location =     {0.7601, 0.9632, 0.4206};
    constexpr VECTOR3 Headlights_Location =     {-0.0056, 0.6552, 2.4872};
    constexpr VECTOR3 Wing_right_Location =     {2.3139, 0.5000, 0.3017};
    constexpr VECTOR3 Wheels_rear_Location =     {-0.0001, 0.1623, -1.1901};
    constexpr VECTOR3 Brake_light_1_Location =     {-0.8478, 0.9643, -2.9591};
    constexpr VECTOR3 Beacon1_Location =     {-4.0374, 0.5003, 0.1162};
    constexpr VECTOR3 Brake_light_2_Location =     {0.8528, 0.9695, -2.9846};
    constexpr VECTOR3 Beacon2_Location =     {4.0428, 0.5003, 0.1162};
    constexpr VECTOR3 Light1_Location =     {-0.9458, 0.6832, 2.5686};
    constexpr VECTOR3 Light2_Location =     {0.9453, 0.6832, 2.5686};
    constexpr VECTOR3 Engine_Location =     {0.0000, 0.0000, -2.2000};
    constexpr VECTOR3 TDP0_Location =     {-0.9402, -0.2410, 2.0700};
    constexpr VECTOR3 TDP3_Location =     {-0.9442, -0.2410, -1.2100};
    constexpr VECTOR3 TDP2_Location =     {0.9358, -0.2410, 2.0700};
    constexpr VECTOR3 TDP4_Location =     {0.9358, -0.2410, -1.2100};
    constexpr VECTOR3 TDP1_Location =     {0.0104, 0.3934, 3.1741};
    constexpr VECTOR3 TDP9_Location =     {0.0104, 1.2434, 0.6741};
    constexpr VECTOR3 TDP5_Location =     {-1.3396, 1.1434, -3.5059};
    constexpr VECTOR3 TDP6_Location =     {1.3304, 1.1434, -3.5059};
    constexpr VECTOR3 TDP7_Location =     {-0.9968, 0.5422, -3.0121};
    constexpr VECTOR3 TDP8_Location =     {0.9950, 0.5422, -3.0121};
    constexpr VECTOR3 TDP_WING2_Location =     {-3.9968, 0.4231, 0.3741};
    constexpr VECTOR3 TDP_WING4_Location =     {3.9981, 0.4231, 0.3741};
    constexpr VECTOR3 TDP_WING1_Location =     {-3.9968, 0.4231, -0.3259};
    constexpr VECTOR3 TDP_WING3_Location =     {3.9981, 0.4231, -0.3259};
    constexpr VECTOR3 TDP10_Location =     {-0.0042, -0.2410, 2.0700};
    constexpr VECTOR3 Rear_Axle_Location =     {-0.0042, -0.2410, -1.2100};
    constexpr VECTOR3 Axis_Left_wing_Location =     {-0.8976, 0.5000, 0.1900};
    constexpr VECTOR3 Axis_Left_wing_fold_Location =     {-2.4328, 0.5781, 0.1571};
    constexpr VECTOR3 Axis_right_wing_fold_Location =     {2.4672, 0.5781, 0.1571};
    constexpr VECTOR3 Axis_wing_door_right_Location =     {0.2026, 1.2397, 0.4426};
    constexpr VECTOR3 Axis_wing_door_left_Location =     {-0.2144, 1.2397, 0.4426};
    constexpr VECTOR3 Axis_Right_wing_Location =     {0.8977, 0.5000, 0.1900};
    constexpr VECTOR3 Axis_elevator_Location =     {-0.0104, 1.1555, -3.5994};
    constexpr VECTOR3 Axis_rudder_left_Location =     {-1.0116, 0.8555, -3.3789};
    constexpr VECTOR3 Axis_rudder_right_Location =     {0.9884, 0.8555, -3.3789};
    constexpr VECTOR3 Axis_aileron_left_Location =     {-3.6826, 0.5002, -0.2380};
    constexpr VECTOR3 Axis_aileron_right_Location =     {3.6810, 0.5002, -0.2380};

#endif
