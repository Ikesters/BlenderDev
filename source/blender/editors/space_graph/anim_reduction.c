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
#include <float.h>

#include "MEM_guardedalloc.h"

#include "DNA_scene_types.h"
#include "DNA_object_types.h"
#include "DNA_anim_types.h"

#include "BLI_blenlib.h"
#include "BLI_math.h"
#include "BLI_math_base.h"
#include "BLI_utildefines.h"

#include "BKE_context.h"
#include "BKE_fcurve.h"
#include "BKE_main.h"
#include "BKE_report.h"
#include "BKE_scene.h"
#include "BKE_screen.h"
#include "BKE_unit.h"
#include "BKE_global.h"
#include "BKE_nla.h"
#include "BKE_library.h"


#include "BLF_translation.h"

#include "RNA_access.h"
#include "RNA_define.h"
#include "RNA_enum_types.h"

#include "WM_api.h"
#include "WM_types.h"

#include "BIF_gl.h"
#include "BIF_glutil.h"

#include "UI_interface.h"
#include "UI_interface_icons.h"
#include "UI_view2d.h"
#include "UI_resources.h"

#include "ED_anim_api.h"
#include "ED_reduction.h"
#include "ED_screen.h"
#include "ED_util.h"
#include "ED_numinput.h"
#include "ED_object.h"
#include "ED_transform.h"
#include "ED_types.h"
#include "ED_keyframing.h"
#include "ED_keyframes_edit.h"
#include "ED_markers.h"

#include "graph_intern.h"



/* little cache for values... */
typedef struct TempFrameValCache {
	float frame, val;
} TempFrameValCache;

typedef struct BezHandle {
	float x, y, z;
} BezHandle;

typedef struct nStop {
    float cost;
    int n;
    int *path;
} nStop;


void copyPath(int *tgt, int *src, int nPts) { // 
    for (int i = 0; i < nPts; i++) {
    	tgt[i] = src[i];
    }
}

void copyPathAndAdd(int * tgt, int * src, int nPts, int toAppend) { //
    copyPath(tgt, src, nPts);
    tgt[nPts] = toAppend;
}

void nStopTableCopy(int nPtsSq, nStop a[nPtsSq], nStop b[nPtsSq]) { //
    for (int i = 0; i < nPtsSq; i++) {
        b[i].cost = a[i].cost;
        b[i].n = a[i].n;

        b[i].path = malloc(b[i].n * sizeof(int)); 
        copyPath(b[i].path, a[i].path, b[i].n);
    }
}

float pointLineDist(BezHandle p, BezHandle q1, BezHandle q2) { //

	float d_q2q1[3];
	d_q2q1[0] = q2.x - q1.x;
	d_q2q1[1] = q2.y - q1.y;
	d_q2q1[2] = q2.z - q1.z;

	float d_q1p[3]; 
	d_q1p[0] = q1.x - p.x;
	d_q1p[1] = q1.y - p.y;
	d_q1p[2] = q1.z - p.z;
	
	float top[3];
	cross_v3_v3v3(top, d_q2q1, d_q1p);

	return normalize_v3(top) / normalize_v3(d_q2q1);
}

float maxChordDistBetween(BezHandle *keyframes, int i, int j) { //
    float maxDist = 0;
    
    BezHandle q1 = keyframes[i];
    BezHandle q2 = keyframes[j];

    for (int k = i; k < j; k++) {
        BezHandle p = keyframes[k];

        float dist = pointLineDist(p, q1, q2);
        if (dist > maxDist) {
            maxDist = dist;
        }
    }

    return maxDist;
}


void initTable (int nPtsSq, nStop table[nPtsSq]) { //
	for (int i = 0; i < nPtsSq; i++) {
		table[i].cost = 99999;
		table[i].n = 0;
	} 
}


void makeZeroStopTable(int nPts, int nPtsSq, nStop e[nPtsSq], BezHandle *keyframes) { //
    for (int i = 0; i < nPts - 1; i++) {
        for (int j = i + 1; j < nPts; j++) {
        	int index = i * nPts + j;
            e[index].cost = maxChordDistBetween(keyframes, i, j);
            e[index].n = 2;

            e[index].path = malloc(e[index].n * sizeof(int));  
            e[index].path[0] = i;
            e[index].path[1] = j;
        }
    }
}


int * makeNStopTable(int nPts, int nPtsSq, int maxN, int n, nStop nTable[nPtsSq], nStop zTable[nPtsSq]) {
    if (n > maxN) {
    	int *path = malloc(nTable[nPts - 1].n * sizeof (int));
    	copyPath(path, nTable[nPts - 1].path, nTable[nPts - 1].n);

    	for (int i = 0; i < nPtsSq; i++) {
    		if (nTable[i].n > 0) {	
    			free(nTable[i].path);
    		}
    	}
        return path;
    }

    nStop nTableNext[nPtsSq];
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


    return makeNStopTable(nPts, nPtsSq, maxN, n + 1, nTableNext, zTable);

}




/* Evaluates the curves between each selected keyframe on each frame, and keys the value  */
void reduce_fcurve(FCurve *fcu, int count)
{
    int i;
	BezTriple *bezt;
    
	if (fcu->bezt == NULL) /* ignore baked */
		return;
	
	/* Store keyframe locations in array */
	BezHandle keyframes[fcu->totvert];

	for (i = 0, bezt = fcu->bezt; i < fcu->totvert; i++, bezt++) {
		BezHandle bzh;
		bzh.x = bezt->vec[1][0];
		bzh.y = bezt->vec[1][1];
		bzh.z = 0;
		keyframes[i] = bzh;

        printf("keyframes -- ");
        printf("x:%2.2f, y:%2.2f\n", bzh.x, bzh.y);
	}



	




	int nPts = fcu->totvert;
	int nPtsSq = nPts * nPts;

	nStop zTable[nPtsSq];
    initTable(nPtsSq, zTable);
    makeZeroStopTable(nPts, nPtsSq, zTable, keyframes);
    
    nStop nTable[nPtsSq];
    initTable(nPtsSq, nTable);
    nStopTableCopy(nPtsSq, zTable, nTable);
    
    int * path = makeNStopTable(nPts, nPtsSq, count - 3, 0, nTable, zTable);
    

    
    for (int i = 0; i < nPts * nPts; i++) {
   		if (zTable[i].n > 0) {
    		free(zTable[i].path);
    	}
    }



	clear_fcurve_keys(fcu);

	for (int i = 0; i < count; i++) {
		int frame = keyframes[path[i]].x;
		float val = keyframes[path[i]].y;
		insert_vert_fcurve(fcu, frame, val, 1);
	}

    calchandles_fcurve(fcu);




    

}

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
	ScrArea *sa = CTX_wm_area(C);
	int retval = OPERATOR_PASS_THROUGH;
	
	/* removed check for Y coord of event, keymap has bounbox now */
	
	/* allow operator to run now */
	if (invoke_func)
		retval = invoke_func(C, op, event);
	else if (op->type->exec)
		retval = op->type->exec(C, op);
	else
		BKE_report(op->reports, RPT_ERROR, "Programming error: operator does not actually have code to do anything!");
		
	/* return status modifications - for now, make this spacetype dependent as above */
	if (sa->spacetype != SPACE_TIME) {
		/* unless successful, must add "pass-through" to let normal operator's have a chance at tackling this event */
		if ((retval & (OPERATOR_FINISHED | OPERATOR_INTERFACE)) == 0) {
			retval |= OPERATOR_PASS_THROUGH;
		}
	}
	
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
	// printf("%s\n", "reduce_fcurve_keys - RICHARD C, begun");
	ListBase anim_data = {NULL, NULL};
	bAnimListElem *ale;
	int filter;
	
	/* filter data */
	filter = (ANIMFILTER_DATA_VISIBLE | ANIMFILTER_CURVE_VISIBLE | ANIMFILTER_FOREDIT | ANIMFILTER_NODUPLIS | ANIMFILTER_SEL);
	ANIM_animdata_filter(ac, &anim_data, filter, ac->data, ac->datatype);
	
	/* loop through filtered data and add keys between selected keyframes on every frame  */
	for (ale = anim_data.first; ale; ale = ale->next) {
		reduce_fcurve((FCurve *)ale->key_data, count);
		ale->update |= ANIM_UPDATE_DEPS;
	}

	ANIM_animdata_update(ac, &anim_data);
	ANIM_animdata_freelist(&anim_data);
	// printf("%s\n", "reduce_fcurve_keys - RICHARD C2, completed");
}

/* reduce selected fcurves */
static int ed_reduction_reduce_exec(bContext *C, wmOperator *op)
{
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
	return ed_reduction_opwrap_invoke_custom(C, op, event, WM_operator_props_popup_confirm);
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
