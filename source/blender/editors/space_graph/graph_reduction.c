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

#include "MEM_guardedalloc.h"

#include "DNA_scene_types.h"
#include "DNA_object_types.h"
#include "DNA_anim_types.h"

#include "BLI_blenlib.h"
#include "BLI_math_base.h"

#include "BKE_context.h"
#include "BKE_fcurve.h"
#include "BKE_main.h"
#include "BKE_report.h"
#include "BKE_scene.h"
#include "BKE_screen.h"
#include "BKE_global.h"

#include "RNA_access.h"
#include "RNA_define.h"
#include "RNA_enum_types.h"

#include "WM_api.h"
#include "WM_types.h"

#include "ED_anim_api.h"
#include "ED_screen.h"
#include "ED_util.h"
#include "ED_numinput.h"
#include "ED_types.h"
#include "ED_keyframing.h"
#include "ED_keyframes_edit.h"
#include "ED_reduction.h"

#include "graph_intern.h"


/* Utilities -------------------------------------------------------------------------------------------------------- */

bool ED_reduction_is_in_array(int val, int *arr, int size)
{
	int i;
	for (i = 0; i < size; i++) {
		if (arr[i] == val)
			return true;
	}
	return false;
}


/* F-Curve Roughness Anaylsis --------------------------------------------------------------------------------------- */

/* Sums the roughness at each point the given fcurve */
double ED_reduction_get_fcurve_roughness(FCurve *fcu)
{
	int npts = fcu->totvert;
	int i;
	BezTriple *bezt;

	/* Store keyframes y values in a temporary vector */
	double vals[npts];
	for (i = 0, bezt = fcu->bezt; i < npts; i++, bezt++) {
		vals[i] = bezt->vec[1][1]; 
	}

	/* Roughness at a point is taken as a (positive) approximation of the second order dervivative */
	double totalRoughness = 0.0;
	double roughness;
	for (i = 0; i < npts - 2; i++) {
		roughness = fabs(vals[i + 2] - 2 * vals[i + 1] + vals[i]);
		totalRoughness += roughness;
	}
	
	return totalRoughness;
}

/* Keyframe Placement Analysis -------------------------------------------------------------------------------------- */

typedef struct Frame {
	double f;
	double v;
} Frame;

double ED_reduction_choord_to_frame_cost(Frame p, Frame q1, Frame q2)
{
	double numer = fabs((q2.f - q1.f) * (q1.v - p.v) - (q1.f - p.f) * (q2.v - q1.v));
	double denom = sqrt(pow(q2.f - q1.f, 2) + pow(q2.v - q1.v, 2));

	if (denom == 0) { return numer; }
	else { return numer / denom; }
}

double ED_reduction_path_cost(Frame *frames, int startF, int endF)
{
	double maxDist = 0;

	for (int i = startF; i < endF; i++) {
		double dist = ED_reduction_choord_to_frame_cost(frames[i], frames[startF], frames[endF]);
		
		if (dist > maxDist)
			maxDist = dist;
	}

	return maxDist;
}

/* NStop Tables ------------------------------------------------------------------------------------------------------ */

typedef struct NStop {
	float cost;
	int n;
	int *path;
} NStop;

void ED_reduction_copy_stoptable_path(int *tgt, int *src, int npts)
{
	for (int i = 0; i < npts; i++) {
		tgt[i] = src[i];
	}
}

void ED_reduction_copy_stoptable_path_and_add(int *tgt, int *src, int npts, int v)
{
	ED_reduction_copy_stoptable_path(tgt, src, npts);
	tgt[npts] = v;
}

void ED_reduction_init_stoptable(int npts_sq, NStop table[npts_sq])
{
	for (int i = 0; i < npts_sq; i++) {
		table[i].cost = 99999;
		table[i].n = 0;
	} 
}

void ED_reduction_copy_stoptable(int npts_sq, NStop a[npts_sq], NStop b[npts_sq])
{
	for (int i = 0; i < npts_sq; i++) {
		b[i].cost = a[i].cost;
		b[i].n = a[i].n;

		b[i].path = malloc(b[i].n * sizeof(int)); 
		ED_reduction_copy_stoptable_path(b[i].path, a[i].path, b[i].n);
	}
}

void ED_reduction_delete_stoptable(int npts_sq, NStop table[npts_sq])
{
	for (int i = 0; i < npts_sq; i++) {
		if (table[i].n > 0) {	
			free(table[i].path);
		}
	}
}

void ED_reduction_zero_stoptable(int npts, int npts_sq, NStop table[npts_sq], Frame *frames)
{
	for (int i = 0; i < npts - 1; i++) {
		for (int j = i + 1; j < npts; j++) {
			
			int index = i * npts + j;
			table[index].cost = ED_reduction_path_cost(frames, i, j);
			table[index].n = 2;

			table[index].path = malloc(table[index].n * sizeof(int));  
			table[index].path[0] = i;
			table[index].path[1] = j;
		}
	}
}

void ED_reduction_n_stoptable(int npts, int npts_sq, int n_stops, int n, NStop nTable[npts_sq], NStop zTable[npts_sq])
{
	if (nTable[npts - 1].n == n_stops)
		return;

	NStop tmp_table[npts_sq];
	ED_reduction_init_stoptable(npts_sq, tmp_table);

	for (int i = 0; i < npts; i++) {
		for (int j = i + n + 1; j < npts; j++) {
			int indexIJ = i * npts + j;

			// Find the least cost path between i and j
			float minCost = 99999;
			for (int k = i + 1; k < j; k++) {
				int indexIK = i * npts + k;
				int indexKJ = k * npts + j;
				float cost = max_ff(nTable[indexIK].cost, zTable[indexKJ].cost);

				// If the path cost is less, update the best cost information
				if (cost < minCost) {
					minCost = cost;	
					tmp_table[indexIJ].cost = cost;
					tmp_table[indexIJ].n = nTable[indexIK].n + 1;
					tmp_table[indexIJ].path = malloc(tmp_table[indexIJ].n * sizeof(int));  
					ED_reduction_copy_stoptable_path_and_add(tmp_table[indexIJ].path, nTable[indexIK].path, nTable[indexIK].n, j);
				}
			}
		}
	}

	ED_reduction_delete_stoptable(npts_sq, nTable);
	ED_reduction_copy_stoptable(npts_sq, tmp_table, nTable);
	ED_reduction_n_stoptable(npts, npts_sq, n_stops, n + 1, nTable, zTable);
}

/* Reduction API ---------------------------------------------------------------------------------------------------- */

int *ED_reduction_pick_best_frames_fcurve(FCurve *fcu, int n_stops)
{
	int i;
	BezTriple *bezt;

	int npts = fcu->totvert;
	int npts_sq = npts * npts;

	/* Extract frame data */
	Frame frames[fcu->totvert];
	for (i = 0, bezt = fcu->bezt; i < fcu->totvert; i++, bezt++) {
		frames[i] = (Frame) { bezt->vec[1][0], bezt->vec[1][1] };
	}

	/* Build stop tables */
	NStop zTable[npts_sq];
	NStop nTable[npts_sq];
	ED_reduction_init_stoptable(npts_sq, zTable);
	ED_reduction_zero_stoptable(npts, npts_sq, zTable, frames);
	ED_reduction_init_stoptable(npts_sq, nTable);
	ED_reduction_copy_stoptable(npts_sq, zTable, nTable);
	ED_reduction_n_stoptable(npts, npts_sq, n_stops, 0, nTable, zTable);

	/* Store frame indicies */
	int *frameIndicies = malloc(nTable[npts - 1].n * sizeof (int));
	ED_reduction_copy_stoptable_path(frameIndicies, nTable[npts - 1].path, nTable[npts - 1].n);
	
	/* Clean up */
	ED_reduction_delete_stoptable(npts_sq, zTable);
	ED_reduction_delete_stoptable(npts_sq, nTable);
	
	return frameIndicies;
}

int *ED_reduction_pick_best_frames_fcurves(ListBase anim_data, int n_stops)
{
	bAnimListElem *ale;
	int i;

	/* Get the index of the roughest curve  */
	double maxRoughness = 0;
	int maxRoughnessIndex = 0;
	for (i = 0, ale = anim_data.first; ale; i++, ale = ale->next) {
		double roughness = ED_reduction_get_fcurve_roughness((FCurve *)ale->key_data);
		if (roughness > maxRoughness) {
			maxRoughness = roughness;
			maxRoughnessIndex = i;
		}
	}

	/* Get the best placement of n keyframes for the roughest curve */
	int * frameIndicies;

	/* ??? How can I access the bAnimListElem at a given index? */
	for (i = 0, ale = anim_data.first; ale; i++, ale = ale->next) {
		if (i == maxRoughnessIndex)
			frameIndicies = ED_reduction_pick_best_frames_fcurve((FCurve *)ale->key_data, n_stops);
	}
	return frameIndicies;
}

void ED_reduction_reduce_fcurve_to_frames(FCurve *fcu, int *frameIndicies, int n_stops)
{
	int i;
	BezTriple *bezt;

	/* Cache data for each frame in given indicies */
	Frame frames[n_stops];
	int index = 0;
	for (i = 0, bezt = fcu->bezt; i < fcu->totvert; i++, bezt++) {
		if (ED_reduction_is_in_array(i, frameIndicies, n_stops)) {
			frames[index] = (Frame) { bezt->vec[1][0], bezt->vec[1][1] };
			index ++;
		}
	}

	/* Delete all keys, and then rebuild curves using cache */
	clear_fcurve_keys(fcu);
	for (i = 0; i < n_stops; i++)
		insert_vert_fcurve(fcu, frames[i].f, frames[i].v, 1);
	calchandles_fcurve(fcu);
}

void ED_reduction_reduce_fcurves_to_frames(ListBase anim_data, int *frameIndicies, int n_stops)
{
	bAnimListElem *ale;

	for (ale = anim_data.first; ale; ale = ale->next) {
		ED_reduction_reduce_fcurve_to_frames((FCurve *)ale->key_data, frameIndicies, n_stops);
		ale->update |= ANIM_UPDATE_DEFAULT;
	}
}

