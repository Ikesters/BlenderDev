/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version. 
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2008 Blender Foundation.
 * All rights reserved.
 *
 * 
 * Contributor(s): Blender Foundation
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/editors/animation/anim_reduction.c
 *  \ingroup ??
 */

#ifdef _MSC_VER
#  pragma warning (disable:4786)
#endif

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// #include "MEM_guardedalloc.h"

#include "DNA_scene_types.h"
#include "DNA_object_types.h"
#include "DNA_anim_types.h"

#include "BLI_blenlib.h"
// #include "BLI_math.h"
#include "BLI_math_base.h"
// #include "BLI_utildefines.h"

#include "BKE_context.h"
#include "BKE_fcurve.h"
#include "BKE_main.h"
#include "BKE_report.h"
#include "BKE_scene.h"
#include "BKE_screen.h"
// #include "BKE_unit.h"
#include "BKE_global.h"
// #include "BKE_nla.h"
// #include "BKE_library.h"


// #include "BLF_translation.h"

#include "RNA_access.h"
#include "RNA_define.h"
#include "RNA_enum_types.h"

#include "WM_api.h"
#include "WM_types.h"

// #include "BIF_gl.h"
// #include "BIF_glutil.h"

// #include "UI_interface.h"
// #include "UI_interface_icons.h"
// #include "UI_view2d.h"
// #include "UI_resources.h"

#include "ED_anim_api.h"
// #include "ED_reduction.h"
#include "ED_screen.h"
#include "ED_util.h"
#include "ED_numinput.h"
// #include "ED_object.h"
// #include "ED_transform.h"
#include "ED_types.h"
#include "ED_keyframing.h"
#include "ED_keyframes_edit.h"
// #include "ED_markers.h"

#include "graph_intern.h"

#include "ED_reduction.h"


typedef struct Frame {
	double f;
	double v;
} Frame;

// typedef struct AnimCurve {
// 	int numFrames;
// 	double energy;
// 	Frame * frames;
// } AnimCurve;

typedef struct NStop {
    float cost;
    int n;
    int * path;
} NStop;

#ifndef max
	#define max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif


/* Tmp API ----------------------------- */

// void logAnimCurve(AnimCurve ac, char * msg) {
// 	printf("\n\nCurve %s, e=%2.2f\n", msg, ac.energy);
// 	for (int i = 0; i < ac.numFrames; i++) {
// 		Frame p = ac.frames[i];
// 		printf("P%d: f:%2.2f v:%2.2f \n", i, p.f, p.v);
// 	}
// }

/* ??? ----------------------------- */


void copyPath(int * tgt, int * src, int nPts) { // 
    for (int i = 0; i < nPts; i++) {
    	tgt[i] = src[i];
    }
}

void copyPathAndAdd(int * tgt, int * src, int nPts, int toAppend) { //
    copyPath(tgt, src, nPts);
    tgt[nPts] = toAppend;
}

void nStopTableCopy(int nPtsSq, NStop a[nPtsSq], NStop b[nPtsSq]) { //
    for (int i = 0; i < nPtsSq; i++) {
        b[i].cost = a[i].cost;
        b[i].n = a[i].n;

        b[i].path = malloc(b[i].n * sizeof(int)); 
        copyPath(b[i].path, a[i].path, b[i].n);
    }
}

double pointLineDist(Frame p, Frame q1, Frame q2) { //	
	double x0 =  p.f;
	double x1 = q1.f;
	double x2 = q2.f;
	
	double y0 =  p.v;
	double y1 = q1.v;
	double y2 = q2.v;

	double numer = fabs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1));
	double denom = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));

	if (denom == 0) { return numer; }
	else { return numer / denom; }
}

float maxChordDistBetween(Frame * frames, int i, int j) { //
    float maxDist = 0;
    
    Frame q1 = frames[i];
    Frame q2 = frames[j];

    for (int k = i; k < j; k++) {
        Frame p = frames[k];

        float dist = pointLineDist(p, q1, q2);
        if (dist > maxDist) {
            maxDist = dist;
        }
    }

    return maxDist;
}

void initTable(int nPtsSq, NStop table[nPtsSq]) { //
	for (int i = 0; i < nPtsSq; i++) {
		table[i].cost = 99999;
		table[i].n = 0;
	} 
}

/* Backend API ----------------------------- */

double energy(Frame p, Frame p1, Frame p2) {
	return (p2.v) - (2 * p1.v) - (p.v);
}

double roughnessOfCurve(FCurve *fcu) {
	int i;
	BezTriple *bezt;
    
	double curveValues[fcu->totvert];
	for (i = 0, bezt = fcu->bezt; i < fcu->totvert; i++, bezt++) {
		curveValues[i] = bezt->vec[1][1];
	}

	double roughness = 0;
	for (int i = 0; i < fcu->totvert - 2; i++) {
		roughness += fabs(curveValues[i + 2] - 2 * curveValues[i + 1] - curveValues[i]);
	}

	return roughness;
}


// void setEnergy(AnimCurve * curve) {
// 	Frame * frames = curve->frames;
// 	curve->energy = 0;

// 	for (int i = 0; i < curve->numFrames - 2; i++) {
// 		double e = energy(frames[i], frames[i + 1], frames[i + 2]);
// 		curve->energy += abs(e);
// 	}
// }

// int findRoughestCurve(AnimCurve * curves, int numCurves) {
// 	double maxEnergy = curves[0].energy;
// 	int roughestIndex = 0;

// 	for (int i = 1; i < numCurves; i++) {
// 		if (curves[i].energy > maxEnergy) {
// 			maxEnergy = curves[i].energy;
// 			roughestIndex = i;
// 		}
// 	}

// 	return roughestIndex;
// }

void makeZeroStopTable(int nPts, int nPtsSq, NStop e[nPtsSq], Frame * frames) { //
    for (int i = 0; i < nPts - 1; i++) {
        for (int j = i + 1; j < nPts; j++) {
        	int index = i * nPts + j;
            e[index].cost = maxChordDistBetween(frames, i, j);
            e[index].n = 2;

            e[index].path = malloc(e[index].n * sizeof(int));  
            e[index].path[0] = i;
            e[index].path[1] = j;
        }
    }
}


int * iterateStopTable(int nPts, int nPtsSq, int maxN, int n, NStop nTable[nPtsSq], NStop zTable[nPtsSq]) {
    if (n > maxN) {
    	int * path = malloc(nTable[nPts - 1].n * sizeof (int));
    	copyPath(path, nTable[nPts - 1].path, nTable[nPts - 1].n);

    	for (int i = 0; i < nPtsSq; i++) {
    		if (nTable[i].n > 0) {	
    			free(nTable[i].path);
    		}
    	}
        return path;
    }

    NStop nTableNext[nPtsSq];
    initTable(nPtsSq, nTableNext);

    for (int i = 0; i < nPts; i++) {
        for (int j = i + n + 1; j < nPts; j++) {

        	int indexIJ = i * nPts + j;

            float minCost = 99999;

            for (int k = i + 1; k < j; k++) {
            	int indexIK = i * nPts + k;
            	int indexKJ = k * nPts + j;

                float cost = max_ff(nTable[indexIK].cost, zTable[indexKJ].cost);
                if (cost < minCost) {
                    minCost = cost;

                    
                    nTableNext[indexIJ].cost = cost;
                    nTableNext[indexIJ].n = nTable[indexIK].n + 1;
                    nTableNext[indexIJ].path = malloc(nTableNext[indexIJ].n * sizeof(int));  

                    copyPathAndAdd(nTableNext[indexIJ].path, nTable[indexIK].path, nTable[indexIK].n, j);

                }
            }
        }
    }

    for (int i = 0; i < nPtsSq; i++) {
    	if (nTable[i].n > 0) {
    		free(nTable[i].path);
    	}
    }


    return iterateStopTable(nPts, nPtsSq, maxN, n + 1, nTableNext, zTable);

}




/* Evaluates the curves between each selected keyframe on each frame, and keys the value  */
int * getSalientFrames(FCurve *fcu, int count)
{
    int i;
	BezTriple *bezt;
	
	/* Store keyframe locations in array */
	Frame frames[fcu->totvert];

	for (i = 0, bezt = fcu->bezt; i < fcu->totvert; i++, bezt++) {
		Frame frame;
		frame.f = bezt->vec[1][0];
		frame.v = bezt->vec[1][1];
		frames[i] = frame;
	}


	int nPts = fcu->totvert;
	int nPtsSq = nPts * nPts;

	NStop zTable[nPtsSq];
    initTable(nPtsSq, zTable);
    makeZeroStopTable(nPts, nPtsSq, zTable, frames);
    
    NStop nTable[nPtsSq];
    initTable(nPtsSq, nTable);
    nStopTableCopy(nPtsSq, zTable, nTable);
    
    int * frameIndicies = iterateStopTable(nPts, nPtsSq, count - 3, 0, nTable, zTable);
    
    
    
    for (int i = 0; i < nPts * nPts; i++) {
   		if (zTable[i].n > 0) {
    		free(zTable[i].path);
    	}
    }

    return frameIndicies;



	

}

// void setSalientFrames(FCurve *fcu, int * salientFrames, int count) {

// 	Frame frames[count];
// 	for (i = 0, bezt = fcu->bezt; i < fcu->totvert; i++, bezt++) {
// 		Frame frame;
// 		frame.f = salientFrames[i];
// 		frame.v = 
// 	}

// 	clear_fcurve_keys(fcu);

	
// 		int frame = salientFrames;
// 		float val = frames[path[i]].v;
// 		insert_vert_fcurve(fcu, frame, val, 1);
// 	}

//     calchandles_fcurve(fcu);
// }

/**
 * Second-tier invoke() callback that performs context validation before running the
 * "custom"/third-tier invoke() callback supplied as the last arg (which would normally
 * be the operator's invoke() callback elsewhere)
 *
 * \param invoke_func "standard" invoke function that operator would otherwise have used.
 * If NULL, the operator's standard exec()
 * callback will be called instead in the appropriate places.
 */
static int ed_reduction_opwrap_invoke_custom(bContext *C, wmOperator *op, const wmEvent *event,
                                             int (*invoke_func)(bContext *, wmOperator *, const wmEvent *))
{
    printf("HI\n");
    
    
	ScrArea *sa = CTX_wm_area(C);
	int retval = OPERATOR_PASS_THROUGH;
	
	/* removed check for Y coord of event, keymap has bounbox now */
	
	/* allow operator to run now */
	if (invoke_func){
        
        retval = invoke_func(C, op, event);
        printf("invoking pop up window\n");
        // exit(0);
    } else if (op->type->exec) {
        printf("executing\n");
        exit(0);
		retval = op->type->exec(C, op);
    } else {
        printf("erroring\n");
        exit(0);
		BKE_report(op->reports, RPT_ERROR, "Programming error: operator does not actually have code to do anything!");
    }
		
	/* return status modifications - for now, make this spacetype dependent as above */
	if (sa->spacetype != SPACE_TIME) {

		/* unless successful, must add "pass-through" to let normal operator's have a chance at tackling this event */
		if ((retval & (OPERATOR_FINISHED | OPERATOR_INTERFACE)) == 0) {

            printf("passing through while waiting for successful\n");
            // exit(0);

			retval |= OPERATOR_PASS_THROUGH;
		}

        // printf("nonsucess\n");
        // exit(0);
	}

    printf("completed call\n");
    // exit(0);
	
	return retval;
}


/* ******************** Reduce Operator *********************** */
/* This operator reduces the number of keyframes used in the selected f-curves. To
 * do this it uses the findSalient function to identify the most important keyframes.
 *
 * Once the findSalient code has complime
 */

/* Evaluates the curves between each selected keyframe on each frame, and keys the value  */
static void reduce_fcurve_keys(bAnimContext *ac, int count)
{	
	printf("%s\n", "reduce_fcurve_keys - RICHARD A, begun");
	ListBase anim_data = {NULL, NULL};
	bAnimListElem *ale;
	int filter;
	
	/* filter data */
	filter = (ANIMFILTER_DATA_VISIBLE | ANIMFILTER_CURVE_VISIBLE | ANIMFILTER_FOREDIT | ANIMFILTER_NODUPLIS | ANIMFILTER_SEL);
	ANIM_animdata_filter(ac, &anim_data, filter, ac->data, ac->datatype);
	
	/* find the roughest curve  */
	double maxE = 0; int maxIndex = 0;
	int i = 0;
	for (ale = anim_data.first; ale; ale = ale->next) {
		double e = roughnessOfCurve((FCurve *)ale->key_data);
		if (e > maxE) { maxE = e; maxIndex = i; }
		i++;

	}

	/* pick salient frames */
	i = 0;
	int * frameIndicies;
	for (ale = anim_data.first; ale; ale = ale->next) {
		if (i == maxIndex) {
			frameIndicies = getSalientFrames((FCurve *)ale->key_data, count);		
		}
		i++;
	}
	
	for (ale = anim_data.first; ale; ale = ale->next) {
		setKeyframes(int * frameIndicies, count);
	}


	// ale->update |= ANIM_UPDATE_DEPS;

	// ANIM_animdata_update(ac, &anim_data);
	// ANIM_animdata_freelist(&anim_data);
}

/* reduce selected fcurves */
static int ed_reduction_reduce_exec(bContext *C, wmOperator *op)
{
    printf("EXECUTING REDUCE!!");
    // exit(0);
	bAnimContext ac;

	/* get editor data */
	if (ANIM_animdata_get_context(C, &ac) == 0) {
		return OPERATOR_CANCELLED;
	}
	
	/* run reduction function */
	int keyCount = RNA_int_get(op->ptr, "KeyCount");
	reduce_fcurve_keys(&ac, keyCount);
	
	/* set notifier that keyframes have changed */
	WM_event_add_notifier(C, NC_ANIMATION | ND_KEYFRAME | NA_EDITED, NULL);
	
	return OPERATOR_FINISHED;
}

static int ed_reduction_reduce_invoke_wrapper(bContext *C, wmOperator *op, const wmEvent *event)
{
	/* now see if the operator is usable */
	return ed_reduction_opwrap_invoke_custom(C, op, event, WM_operator_props_popup);
}



static void REDUCTION_OT_reduce(wmOperatorType *ot)
{
	/* identifiers */
	ot->name = "Reduce F-curve";
	ot->description = "Reduce the number of keyframes in the active f-curve";
	ot->idname = "REDUCTION_OT_reduce";
	
	/* api callbacks */
	ot->invoke = ed_reduction_reduce_invoke_wrapper;
	ot->exec = ed_reduction_reduce_exec;
	ot->poll = graphop_editable_keyframes_poll;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
	
	/* properties */
	ot->prop = RNA_def_int(ot->srna, "KeyCount", 3, 3, 100, "Number of Keys", "How many keyframes to reduce to.", 3, 100);
}



void ED_operatortypes_reduction(void)
{
	WM_operatortype_append(REDUCTION_OT_reduce);
}
