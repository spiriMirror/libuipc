#pragma once
/*****************************************************************/ /**
 * \file   distance.h
 * \brief  distance calculation for point/edge/triangle
 * 
 * \author MuGdxy
 * \date   2024.8
 * \details
 * 
 * All files in directory `./distance/` is referenced from the following repository:
 * 
 * https://github.com/ipc-sim/Codim-IPC/tree/main/Library/Math/Distance
 * 
 * - Thanks to the original author for the great work.
 * 
 * - To make it more readable, logically consistent and GPU compatible, the original code is modified.
 * 
 * - The original code is licensed under the Apache License, Version 2.0.
 * 
 * NOTE:
 * 
 * To reduce the numerical error, the squared distance is used in the calculation.
 * So all the distance calculation functions return the squared distance (usually named as `D`).
 * 
 *********************************************************************/


#include "distance/distance_flagged.h"
#include "distance/edge_edge_mollifier.h"
#include "distance/ccd.h"
