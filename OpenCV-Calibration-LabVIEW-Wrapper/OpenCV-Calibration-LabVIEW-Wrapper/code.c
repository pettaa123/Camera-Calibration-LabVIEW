/* Call Library source file */

#include "extcode.h"

/* lv_prolog.h and lv_epilog.h set up the correct alignment for LabVIEW data. */
#include "lv_prolog.h"

/* Typedefs */

typedef struct {
	int32_t dimSizes[3];
	uint16_t elt[1];
	} TD1;
typedef TD1 **TD1Hdl;

typedef struct {
	int32_t XPx;
	int32_t YPx;
	} TD3;

typedef struct {
	int32_t dimSizes[2];
	TD3 Cluster[1];
	} TD2;
typedef TD2 **TD2Hdl;

typedef struct {
	double XPx;
	double YPx;
	} TD5;

typedef struct {
	int32_t dimSizes[2];
	TD5 Cluster[1];
	} TD4;
typedef TD4 **TD4Hdl;

#include "lv_epilog.h"

int32_t extractCorners(TD1Hdl images, TD2Hdl maskPolygons, 
	TD4Hdl extractedPoints);

int32_t extractCorners(TD1Hdl images, TD2Hdl maskPolygons, 
	TD4Hdl extractedPoints)
{

	/* Insert code here */

}

