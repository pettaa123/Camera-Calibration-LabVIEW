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
	int32 dimSizes[2];
	uint16_t elt[1];
} Arr2D_U16;
typedef Arr2D_U16** Arr2D_U16Hdl;

typedef struct {
	int32 dimSize;
	float elt[1];
} Arr_SGL;
typedef Arr_SGL** Arr_SGLHdl;

typedef struct {
	int32 dimSize;
	double elt[1];
} Arr_DBL;
typedef Arr_DBL** Arr_DBLHdl;

typedef struct {
	int32 dimSizes[2];
	double elt[1];
} Arr2D_DBL;
typedef Arr2D_DBL** Arr2D_DBLHdl;

typedef struct {
	int32 dimSizes[3]; //ToDo: make elements of type Arr2D_U16
	uint16_t elt[1];
} Arr3D_U16;
typedef Arr3D_U16** Arr3D_U16Hdl;

typedef struct {
	int32_t x;
	int32_t y;
} PointXY;

typedef struct {
	int32_t dimSize;
	PointXY Cluster[1];
} Arr_ClusterPointXY;
typedef Arr_ClusterPointXY** Arr_ClusterPointXYHdl;

typedef struct {
	int32_t dimSizes[2];
	PointXY Cluster[1];
} Arr2D_ClusterPointXY;
typedef Arr2D_ClusterPointXY** Arr2D_ClusterPointXYHdl;

typedef struct {
	float x;
	float y;
} PointXYf;

typedef struct {
	int32_t dimSize;
	PointXYf Cluster[1];
} Arr_ClusterPointXYf;
typedef Arr_ClusterPointXYf** Arr_ClusterPointXYfHdl;

typedef struct {
	int32_t dimSizes[2];
	PointXYf Cluster[1];
} Arr2D_ClusterPointXYf;
typedef Arr2D_ClusterPointXYf** Arr2D_ClusterPointXYfHdl;

typedef struct {
	int32 dimSizes[2];
	uint32_t elt[1];
} Arr2D_U32;
typedef Arr2D_U32** Arr2D_U32Hdl;

typedef struct {
	int32 dimSizes[3];
	uint32_t elt[1]; //ToDo: Make this 1D array of type Arr2D
} Arr3D_U32;
typedef Arr3D_U32** Arr3D_U32Hdl;

#include "lv_epilog.h"

#if IsOpSystem64Bit
#define uPtr uQ //unsigned Quad aka 64-bit
#else
#define uPtr uL //unsigned Long aka 32-bit
#endif

/*user defined error codes
5009 -> calibrateCamera: insufficient number of extracted points sets
5010 -> calibrateCamera: dimension error intrinsics
5011 -> calibrateCamera: dimension error distCoeffs
*/

extern "C" OPENCVCALIBDLL_API int32_t setBoardSettings(const uint32_t boardType, const uint32_t cbRows, const uint32_t cbCols, const float squareSize);

extern "C" OPENCVCALIBDLL_API int32_t extractCorners(const Arr2D_U16Hdl image, const Arr_ClusterPointXYHdl maskPolygon, \
Arr_ClusterPointXYfHdl extractedPoints, Arr3D_U32Hdl imagesExtracted, int adaptiveThreshBlockSize, int findMethod);

extern "C" OPENCVCALIBDLL_API int32_t calibrateCamera(const Arr2D_ClusterPointXYfHdl extractedPoints, int flags, Arr2D_DBLHdl intrinsics, Arr_DBLHdl distCoeffs, Arr_SGLHdl reprojErrs, double &totalAvgErr);

extern "C" OPENCVCALIBDLL_API int32_t undistort(const Arr2D_U16Hdl inOutput, Arr2D_DBLHdl cameraMatrix, Arr_DBLHdl	distCoeffs);

/*extern "C" OPENCVCALIBDLL_API int32_t extractCornersMulti(const Arr3D_U16Hdl images, const Arr2D_ClusterPointXYHdl maskPolygons, \
	const uint32_t cbRows,const uint32_t cbCols, Arr2D_ClusterPointXYfHdl extractedPoints, Arr3D_U32Hdl imagesExtracted);*/
//extern "C" OPENCVCALIBDLL_API int32_t calibrateCamera(const Arr3D_U16Hdl images,const Arr3D_DBLHdl masks, Arr3D_DBLHdl extractedPoints);


