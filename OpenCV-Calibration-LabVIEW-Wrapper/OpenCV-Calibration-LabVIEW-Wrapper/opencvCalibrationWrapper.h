#pragma once


#ifdef OPENCVCALIBRATIONLABVIEWWRAPPER_EXPORTS
#define OPENCVCALIBDLL_API __declspec(dllexport)
#else
#define OPENCVCALIBDLL_API __declspec(dllimport)
#endif


#include "fundtypes.h"
#include "extcode.h"

#include "lv_prolog.h"
#include "platdefines.h"

/* LabVIEW created typedef */
typedef struct {
	int32 dimSizes[3];
	uint16_t elt[1];
} Arr3D_U16;
typedef Arr3D_U16** Arr3D_U16Hdl;

typedef struct {
	int32_t x;
	int32_t y;
} PointXY;

typedef struct {
	int32_t dimSizes[2];
	PointXY Cluster[1];
} Arr_ClusterPointXY;
typedef Arr_ClusterPointXY** Arr_ClusterPointXYHdl;

typedef struct {
	int32_t dimSizes;
	PointXY Cluster[1];
} ClusterPointXY;
typedef ClusterPointXY** ClusterPointXYHdl;

typedef struct {
	float x;
	float y;
} PointXYf;

typedef struct {
	int32_t dimSizes[2];
	PointXYf Cluster[1];
} Arr_ClusterPointXYf;
typedef Arr_ClusterPointXYf** Arr_ClusterPointXYfHdl;



#include "lv_epilog.h"

#if IsOpSystem64Bit
#define uPtr uQ //unsigned Quad aka 64-bit
#else
#define uPtr uL //unsigned Long aka 32-bit
#endif


extern "C" OPENCVCALIBDLL_API int32_t extractCorners(const Arr3D_U16Hdl images, const Arr_ClusterPointXYHdl maskPolygons,const uint32_t cbRows,const uint32_t cbCols, Arr_ClusterPointXYfHdl extractedPoints);
//extern "C" OPENCVCALIBDLL_API int32_t calibrateCamera(const Arr3D_U16Hdl images,const Arr3D_DBLHdl masks, Arr3D_DBLHdl extractedPoints);


