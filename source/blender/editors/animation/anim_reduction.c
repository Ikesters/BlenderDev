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

#include "MEM_guardedalloc.h"

#include "DNA_scene_types.h"
#include "DNA_object_types.h"

#include "BLI_blenlib.h"
#include "BLI_math_base.h"
#include "BLI_utildefines.h"

#include "BKE_context.h"
#include "BKE_fcurve.h"
#include "BKE_main.h"
#include "BKE_report.h"
#include "BKE_scene.h"
#include "BKE_screen.h"
#include "BKE_unit.h"

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

/* reudction active fcurve */
static int ed_reduction_reduce_exec(bContext *C, wmOperator *op)
{
	// TimeMarker *marker = ED_markers_get_first_selected(ED_context_get_markers(C));

	// if (marker) {
	// 	RNA_string_get(op->ptr, "name", marker->name);
		
	// 	WM_event_add_notifier(C, NC_SCENE | ND_MARKERS, NULL);
	// 	WM_event_add_notifier(C, NC_ANIMATION | ND_MARKERS, NULL);
		
	// 	return OPERATOR_FINISHED;
	// }
	// else {
	// 	return OPERATOR_CANCELLED;
	// }

	printf("%s\n", "ed_reduction_reduce_exec - RICHARD A");
	return OPERATOR_FINISHED;
}

static int ed_reduction_reduce_invoke_wrapper(bContext *C, wmOperator *op, const wmEvent *event)
{
	/* must initialize the marker name first if there is a marker selected */
	// TimeMarker *marker = ED_markers_get_first_selected(ED_context_get_markers(C));
	// if (marker)
	// 	RNA_string_set(op->ptr, "name", marker->name);
	
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
	ot->poll = ED_operator_graphedit_active;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
	
	/* properties */
	// ot->prop = RNA_def_string(ot->srna, "name", "RenamedMarker", sizeof(((TimeMarker *)NULL)->name), "Name", "New name for marker");
}



void ED_operatortypes_marker(void)
{
	WM_operatortype_append(REDUCTION_OT_reduce);
}
