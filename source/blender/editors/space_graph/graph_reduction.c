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
#include "BLI_math_vector.h"

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
/* 
 * Various utility functions, nothing interesting to see here.
 */

void ED_reduction_copy_indicies(int *tgt, int *src, int npts)
{
	memcpy(tgt, src, sizeof(int) * npts);
}

void ED_reduction_copy_indicies_and_add(int *tgt, int *src, int npts, int v)
{
	ED_reduction_copy_indicies(tgt, src, npts);
	tgt[npts] = v;
}

bool ED_reduction_val_in_array(int val, int *arr, int size)
{
	int i;

	for (i = 0; i < size; i++) {
		if (arr[i] == val)
			return true;
	}
	return false;
}

int ED_reduction_get_number_of_frames(ListBase *anim_data)
{
	bAnimListElem *ale;
	FCurve * fcu;

	for (ale = anim_data->first; ale; ale = ale->next) {
		fcu = ale->key_data;
		return fcu->totvert;
	}

	return -1;
}


/* Pose Array Construction ------------------------------------------------------------------------------------------ */
/* 
 * Instead of running a cost function on n one-dimensional f-curves, we choose to use 1 n-dimensional curve. The
 * following functions create, fill, and delete this data structure. This n-dimensional curve can be imagined as an
 * array of poses, where a pose is defined as the current value for each degree of freedeom (the value for each 
 * motion curve).
 */

void ED_reduction_init_pose_arr(NPoseArr *n_pose_arr, int n_frames, int n_curves)
{
	int i;

	(*n_pose_arr) = MEM_callocN(sizeof(float *) * n_frames, "NPoseArr_new");
	for (i = 0; i < n_frames; i++)
		(*n_pose_arr)[i] = MEM_callocN(sizeof(float) * n_curves, "NPoseArr_row");
}



void ED_reduction_fill_pose_arr(NPoseArr *n_pose_arr, ListBase *anim_data, int n_frames)
{ 
	bAnimListElem *ale;
	BezTriple *bezt;
	FCurve * fcu;
	int i, j;
	
	for (j = 0; j < n_frames; j++)
		(*n_pose_arr)[j][0] = j;

	for (i = 0, ale = anim_data->first; ale; i++, ale = ale->next) {
		fcu = ale->key_data;

		for (j = 0, bezt = fcu->bezt; j < fcu->totvert; j++, bezt++)
			(*n_pose_arr)[j][i + 1] = bezt->vec[1][1];
	}
}

void ED_reduction_free_pose_arr(NPoseArr *n_pose_arr, int n_frames)
{
	int i;

	for (i = 0; i < n_frames; i++)
		MEM_freeN((*n_pose_arr)[i]);
	MEM_freeN((*n_pose_arr));
}


/* Cost Analysis ---------------------------------------------------------------------------------------------------- */
/* 
 * The following functions are used when evaluating how successful a proposed reduction is. A proposed reduction is
 * composed of segments, where each segments is defined by a start frame, a finish frame, and a path. The cost of the
 * segment is taken to be the maximum perpendicular distance between the original f-curve and the multi-point path
 * (each pair of points in the path is referred to as a "chord").
 */

float ED_reduction_line_to_point_dist(float *p, float *q1, float *q2, const int npts)
{
	float q_p1[npts], q2_q1[npts], p_q1q2[npts];
	float t, numer, denom;

	sub_vn_vnvn(q_p1, p, q1, npts);
	sub_vn_vnvn(q2_q1, q2, q1, npts);

	numer = dot_vn_vn(q_p1, q2_q1, npts);
	denom = dot_vn_vn(q2_q1, q2_q1, npts);
	if (denom != 0) {
		t = numer / denom;
	} else {
		t = numer;
	}

	mul_vn_fl(q2_q1, npts, t);
	sub_vn_vnvn(p_q1q2, q_p1, q2_q1, npts);

	return sqrt(len_squared_vn(p_q1q2, npts));
}

float ED_reduction_segment_cost(NPoseArr *n_pose_arr, int start_f, int end_f, int n_curves)
{
	float max_dist, dist;
	int i;

	max_dist = 0;
	for (i = start_f; i < end_f; i++) {
		dist = ED_reduction_line_to_point_dist((*n_pose_arr)[i], (*n_pose_arr)[start_f], (*n_pose_arr)[end_f], n_curves);
		
		if (dist > max_dist)
			max_dist = dist;
	}
	return max_dist;
}

/* Reduction Algorithm ---------------------------------------------------------------------------------------------- */
/* 
 * A dynamic programming algorithm is used to try all possible placements of the desired number of keyframes. An 
 * n-dimensional curve is given, which is "learned" by the algorithm. This learning establishes two 
 * stoptables. The best placement of the keyframes can be read directly from the finished tables.
 *
 * A stoptable is the name we have given to the dynamic programming tables used by this algoirthm. Each cell of a 
 * stoptable represents the best path between two points, where the starting point is denoted by the row number, and the 
 * finishing point the column number. Here the word "point" is used to denote a component of the n-dimensional curve (an 
 * n-dimensional point). A "stop" is a point that is placed between the start and finish points. The end goal of the 
 * algorithm is to find the best n-2 stops between the start and finish frames of the motion curve (-2 because the start
 * and finish points are already given).
 *
 * The first table the algorithm computes is called the zero-stoptable (z_table). Here the zero indicates that each
 * cell will feature the best zero stop path between the two points. The other table created is called the n-table,
 * where "n" suggests that each cell features the best n stop path between the two points. The n-table is created by 
 * duplicated the z-table first, and then is recursively updated. Note that the first time it is updated, the n-table
 * will describe the best 1-stop path between each pair of points, or the best three keyframes that start and finish on
 * that pair of points. The second update will describe the best 2-stop path (2 keyframes between), third update 
 * 3-stop path (3 between), and so on.
 *
 * The algorithm finishes when n-2 updates have been completed, at which time the best placements of n keyframes can be 
 * read straight from the n-table.
 */

void ED_reduction_init_stoptable(StopTable *table, int n_frames, int n_keys)
{
	int i;

	*table = MEM_mallocN(sizeof(NStop) * n_frames * n_frames, "StopTable_new");
	for (i = 0; i < n_frames * n_frames; i++) {
		(*table)[i].cost = 99999;
		(*table)[i].n = 0;
		(*table)[i].path = MEM_mallocN(sizeof(int) * n_keys, "Indicies_new");
	}
}

void ED_reduction_copy_stoptable(StopTable a, StopTable b, int n_frames)
{
	int i;

	for (i = 0; i < n_frames * n_frames; i++) {
		b[i].cost = a[i].cost;
		b[i].n = a[i].n;
		ED_reduction_copy_indicies(b[i].path, a[i].path, b[i].n);
	}
}

void ED_reduction_delete_stoptable(StopTable *table, int n_frames)
{
	int i;

	for (i = 0; i < n_frames * n_frames; i++) {
		MEM_freeN((*table)[i].path);
	}
	MEM_freeN(*table);
}

void ED_reduction_zero_stoptable(StopTable table, NPoseArr *n_pose_arr, int n_frames, int n_curves)
{
	int i, j, index;
	float cost;

	for (i = 0; i < n_frames - 1; i++) {
		for (j = i + 1; j < n_frames; j++) {

			index = i * n_frames + j;
			cost = ED_reduction_segment_cost(n_pose_arr, i, j, n_curves);
			table[index].cost = cost;
			table[index].n = 2;
			table[index].path[0] = i;
			table[index].path[1] = j;
		}
	}
}

void ED_reduction_n_stoptable(int n_frames, int n_keys, int n, StopTable n_table,
	    													   StopTable n_tableBuffer,
	    													   StopTable z_table)
{
	int i, j, k, ij, ik, kj;
	float cost_min, cost;

	if (n_table[n_frames - 1].n == n_keys)
		return;

	for (i = 0; i < n_frames; i++) {
		for (j = i + n + 1; j < n_frames; j++) {
			ij = i * n_frames + j;

			/* Find the least cost path between i and j */
			cost_min = 99999;
			for (k = i + 1; k < j; k++) {
				ik = i * n_frames + k;
				kj = k * n_frames + j;
				cost = max_ff(n_table[ik].cost, z_table[kj].cost);

				if (cost < cost_min) {
					cost_min = cost;	
					n_tableBuffer[ij].cost = cost;
					n_tableBuffer[ij].n = n_table[ik].n + 1;
					ED_reduction_copy_indicies_and_add(n_tableBuffer[ij].path, n_table[ik].path, n_table[ik].n, j);
				}
			}
		}
	}

	ED_reduction_n_stoptable(n_frames, n_keys, n + 1, n_tableBuffer, n_table, z_table);
}

/* Bezier Handle Tweaking ------------------------------------------------------------------------------------------- */
/*
 * This algorithm tries a set of different bezier-handle placements for the new keyframes. If a placement that matches
 * the original curve more closely is found, it is kept.
 *
 * This algorithms works by first posing the f-curve as a set of segments, where a segment is defined as any part of the
 * curve that lies between two key-frames. Forty tweaks are tested for right-most anchor of the keyframe to the left, 
 * and another forty for the left-most anchor of the keyframe to the right. These first tweaks will test the
 * handle at half the width of the section below the keyframe, and the last half the width above.
 */

float ED_reduction_interpolation_at(float f, float start_f, float end_f, Anchor anchors)
{
	float numer, denom, t;

	numer = f - start_f;
	denom = end_f - start_f;
	t = denom != 0.0 ? numer / denom : numer;

	return                 pow(1 - t, 3) * start_f    + 
	       3 *     t     * pow(1 - t, 2) * anchors.p1 +
           3 * pow(t, 2) *    (1 - t   ) * anchors.p2 +
               pow(t ,3)                 * end_f;
}

float ED_reduction_interpolation_cost(Frame *org_frames, float start_f, float end_f, Anchor anchors)
{
	float i, original_v, interped_v, cost, cost_max;
	int index;

	cost_max = 0;
	index = 0;
	for (i = start_f; i < end_f; i += 1.0) {
		original_v = org_frames[index].v;
		interped_v = ED_reduction_interpolation_at(i, start_f, end_f, anchors);

		cost = fabs(original_v - interped_v);
		if (cost > cost_max)
			cost_max = cost;

		index ++;
	}

	return cost_max;
}

Anchor ED_reduction_pick_anchor_for_segment(Frame *org_frames, float start_f, float end_f)
{
	float half_segment_length, inc;
	float i, j, bestI, bestJ;
	float cost_min, cost;

	half_segment_length = (end_f - start_f) / 2.0;
	inc = half_segment_length / 20.0;
	cost_min = 99999.0;
	bestI = 0.0;
	bestJ = 0.0;

	for (i = -half_segment_length; i < half_segment_length; i += inc) {
		for (j = -half_segment_length; j < half_segment_length; j += inc) {

			cost = ED_reduction_interpolation_cost(org_frames, start_f, end_f, (Anchor) { i, j });
			if (cost < cost_min) {
				cost_min = cost;
				bestI = i;
				bestJ = j;
			}
		}
	}

	return (Anchor) { bestI, bestJ };
}

void ED_reduction_tweak_fcurve_anchors(FCurve * fcu, Frame *org_frames, Frame *reduced_frames)
{
	int i;
	float start_f, end_f;
	BezTriple *bezLeft;
	BezTriple *bezRight;

	for (i = 1; i < fcu->totvert; i++) {
		start_f = reduced_frames[i - 1].f;
		end_f = reduced_frames[i].f;
		Anchor anchor = ED_reduction_pick_anchor_for_segment(org_frames, start_f, end_f);
		
		bezLeft  = (fcu->bezt + i - 1);
		bezRight = (fcu->bezt + i);
		bezRight->vec[0][1] = anchor.p1;
		bezLeft->vec[2][1] = anchor.p2;
	}
}



/* Reduction API ---------------------------------------------------------------------------------------------------- */
/*
 * Welcome! This module provides two algorithms used by the "Reduce Keyframes" wmOperator (GRAPH_OT_reduce). The module 
 * is still under-development, but should working relatively as intended. Please contact Riro (Richard Roberts) for any
 * questions, issues, or feedback - rykardo.r@gmail.com - thanks!
 *
 * These functions are called directly by the operator. The first function runs the reduction algorithm to find the best
 * n keyframes that represented the given animation. The second function first reduces the keyframes of each motion
 * curve to those indicated by the given indices, and then runs the bezier handle tweaking algorithm.
 */
 
void ED_reduction_pick_best_frames(NPoseArr n_pose_arr, int n_keys, int n_frames, int n_curves, int *indices)
{	
	StopTable z_table = NULL;
	StopTable n_table = NULL;
	StopTable n_tableBuffer = NULL;

	/* Build dynamic-programming tables */
	ED_reduction_init_stoptable(&z_table, n_frames, n_keys);
	ED_reduction_init_stoptable(&n_table, n_frames, n_keys);
	ED_reduction_init_stoptable(&n_tableBuffer, n_frames, n_keys);
	ED_reduction_zero_stoptable(z_table, &n_pose_arr, n_frames, n_curves);
	ED_reduction_copy_stoptable(z_table, n_table, n_frames);

	/* Recursively find the best point-path the first and last frame. */
	ED_reduction_n_stoptable(n_frames, n_keys, 0, n_table, n_tableBuffer, z_table);

	/* Save indices of the best frames */
	ED_reduction_copy_indicies(indices, n_table[n_frames - 1].path, n_table[n_frames - 1].n);
	
	/* Clean up */
	ED_reduction_delete_stoptable(&z_table, n_frames);
	ED_reduction_delete_stoptable(&n_table, n_frames);
	ED_reduction_delete_stoptable(&n_tableBuffer, n_frames);
	ED_reduction_free_pose_arr(&n_pose_arr, n_frames);
}

void ED_reduction_reduce_fcurve(FCurve * fcu, int *indices, int n_frames, int n_keys)
{
	BezTriple *bezt;
	int i, index;

	Frame tmpFrame;
	Frame *org_frames = MEM_mallocN(n_frames * sizeof(Frame), "FrameCache");
	Frame *reduced_frames = MEM_mallocN(n_keys * sizeof(Frame), "FrameCache");

	/* Cache frame information */
	index = 0;
	for (i = 0, bezt = fcu->bezt; i < n_frames; i++, bezt++) {
		tmpFrame.f = bezt->vec[1][0];
		tmpFrame.v = bezt->vec[1][1];
		org_frames[i] = tmpFrame;

		if (ED_reduction_val_in_array(i, indices, n_keys)) {
			reduced_frames[index] = tmpFrame;
			index ++;
		}
	}

	/* Delete all the existing keyframes, adding only the best ones back in */
	clear_fcurve_keys(fcu);
	for (i = 0; i < n_keys; i++)
		insert_vert_fcurve(fcu, reduced_frames[i].f, reduced_frames[i].v, 1);
	calchandles_fcurve(fcu);

	/* Tweaking the bezier handles of the new keyframes to best repliciate the cached data */
	ED_reduction_tweak_fcurve_anchors(fcu, org_frames, reduced_frames);

	/* Clean up */
	MEM_freeN(org_frames);
	MEM_freeN(reduced_frames);
}

