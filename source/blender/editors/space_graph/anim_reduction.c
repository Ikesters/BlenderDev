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
 *  \ingroup edanimation
 */

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



/* Evaluates the curves between each selected keyframe on each frame, and keys the value  */
void reduce_fcurve(FCurve *fcu)
{
	printf("%s\n", "reduce_fcurve_keys - RICHARD D, begun");
	BezTriple *bezt, *start = NULL, *end = NULL;
	TempFrameValCache *value_cache, *fp;
	int sfra, range;
	int i, n, nIndex;

	if (fcu->bezt == NULL) /* ignore baked */
		return;
	
	/* find selected keyframes... once pair has been found, add keyframes  */
	for (i = 0, bezt = fcu->bezt; i < fcu->totvert; i++, bezt++) {

		// printf("    %2.2f, %2.2f\n", bezt->vec[0][0], bezt->vec[0][1]);
		printf("    %2.2f, %2.2f\n", bezt->vec[1][0], bezt->vec[1][1]);
		// printf("    %2.2f, %2.2f\n", bezt->vec[2][0], bezt->vec[2][1]);


		/* check if selected, and which end this is */
		// if (BEZSELECTED(bezt)) {
		// 	if (start) {
		// 		/* set end */
		// 		end = bezt;
				
		// 		/* cache values then add keyframes using these values, as adding
		// 		 * keyframes while sampling will affect the outcome...
		// 		 *	- only start sampling+adding from index=1, so that we don't overwrite original keyframe
		// 		 */
		// 		range = (int)(ceil(end->vec[1][0] - start->vec[1][0]));
		// 		sfra = (int)(floor(start->vec[1][0]));
				
		// 		if (range) {
		// 			value_cache = MEM_callocN(sizeof(TempFrameValCache) * range, "IcuFrameValCache");
					
		// 			/* sample values */
		// 			for (n = 1, fp = value_cache; n < range && fp; n++, fp++) {
		// 				fp->frame = (float)(sfra + n);
		// 				fp->val = evaluate_fcurve(fcu, fp->frame);
		// 			}
					
		// 			/* add keyframes with these, tagging as 'breakdowns' */
		// 			for (n = 1, fp = value_cache; n < range && fp; n++, fp++) {
		// 				nIndex = insert_vert_fcurve(fcu, fp->frame, fp->val, 1);
		// 				BEZKEYTYPE(fcu->bezt + nIndex) = BEZT_KEYTYPE_BREAKDOWN;
		// 			}
					
		// 			/* free temp cache */
		// 			MEM_freeN(value_cache);
					
		// 			/* as we added keyframes, we need to compensate so that bezt is at the right place */
		// 			bezt = fcu->bezt + i + range - 1;
		// 			i += (range - 1);
		// 		}
				
		// 		/* bezt was selected, so it now marks the start of a whole new chain to search */
		// 		start = bezt;
		// 		end = NULL;
		// 	}
		// 	else {
		// 		/* just set start keyframe */
		// 		start = bezt;
		// 		end = NULL;
		// 	}
		// }
	}
	
	/* recalculate channel's handles? */
	// calchandles_fcurve(fcu);
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


/* ******************** Sample Keyframes Operator *********************** */
/* This operator 'bakes' the values of the curve into new keyframes between pairs
 * of selected keyframes. It is useful for creating keyframes for tweaking overlap.
 */

/* Evaluates the curves between each selected keyframe on each frame, and keys the value  */
static void reduce_fcurve_keys(bAnimContext *ac)
{	
	printf("%s\n", "reduce_fcurve_keys - RICHARD C, begun");
	ListBase anim_data = {NULL, NULL};
	bAnimListElem *ale;
	int filter;
	
	/* filter data */
	filter = (ANIMFILTER_DATA_VISIBLE | ANIMFILTER_CURVE_VISIBLE | ANIMFILTER_FOREDIT | ANIMFILTER_NODUPLIS);
	ANIM_animdata_filter(ac, &anim_data, filter, ac->data, ac->datatype);
	
	/* loop through filtered data and add keys between selected keyframes on every frame  */
	for (ale = anim_data.first; ale; ale = ale->next) {
		reduce_fcurve((FCurve *)ale->key_data);
		ale->update |= ANIM_UPDATE_DEPS;
	}

	// ANIM_animdata_update(ac, &anim_data);
	ANIM_animdata_freelist(&anim_data);
	printf("%s\n", "reduce_fcurve_keys - RICHARD C2, completed");
}


/* reudction active fcurve */
static int ed_reduction_reduce_exec(bContext *C, wmOperator *op)
{
	printf("%s\n", "ed_reduction_reduce_exec - RICHARD A, begun");

	bAnimContext ac;

	/* get editor data */
	if (ANIM_animdata_get_context(C, &ac) == 0) {
		return OPERATOR_CANCELLED;
	}
	
	/* sample keyframes */
	reduce_fcurve_keys(&ac);
	
	/* set notifier that keyframes have changed */
	WM_event_add_notifier(C, NC_ANIMATION | ND_KEYFRAME | NA_EDITED, NULL);
	
	printf("%s\n", "ed_reduction_reduce_exec - RICHARD A, OPERATOR_FINISHED");
	return OPERATOR_CANCELLED;
}

static int ed_reduction_reduce_invoke_wrapper(bContext *C, wmOperator *op, const wmEvent *event)
{
	/* must initialize the marker name first if there is a marker selected */
	// TimeMarker *marker = ED_markers_get_first_selected(ED_context_get_markers(C));
	// if (marker)
	// RNA_string_set(op->ptr, "name", marker->name);
	
	// /* now see if the operator is usable */

	printf("%s\n", "ed_reduction_invoke_wrapper - RICHARD B");
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
	// ot->poll = ED_operator_graphedit_active;
	ot->poll = graphop_editable_keyframes_poll;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
	
	/* properties */
	ot->prop = RNA_def_int(ot->srna, "count", 8, 2, 100, "Count", "How many keyframes to reduce to.", 2, 100);
}



void ED_operatortypes_reduction(void)
{
	WM_operatortype_append(REDUCTION_OT_reduce);
}
