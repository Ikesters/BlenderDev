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
#include "ED_keyframes_reduction.h"


/* Utilities -------------------------------------------------------------------------------------------------------- */
/* 
 * Various utility functions, nothing interesting to see here.
 */

void ED_reduction_copy_indicies(int *tgt, int *src, int npts)
{
	memcpy(tgt, src, sizeof(int) * npts);
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
	FCurve *fcu;

	ale = anim_data->first;
	fcu = ale->key_data;
	return fcu->totvert;
}


/* Pose Array Construction ------------------------------------------------------------------------------------------ */
/* 
 * Instead of running a cost function on n one-dimensional fcurves, we choose to use 1 n-dimensional curve. The
 * following functions create, fill, and delete this data structure. This n-dimensional curve can be imagined as an
 * array of poses, where the component of that array at index i contains the pose for frame i. The pose itself is an 
 * of the value of each fcurve at frame i.
 */

void ED_reduction_init_pose_arr(NPoseArr *n_pose_arr, int n_frames, int n_curves)
{
	int i;

	(*n_pose_arr) = MEM_mallocN(sizeof(float *) * n_frames, "NPoseArr_new");
	for (i = 0; i < n_frames; i++)
		(*n_pose_arr)[i] = MEM_mallocN(sizeof(float) * n_curves, "NPoseArr_row");
}



void ED_reduction_fill_pose_arr_beziertriples(NPoseArr *n_pose_arr, ListBase *anim_data, int n_frames)
{ 
	bAnimListElem *ale;
	FCurve *fcu;
	BezTriple *bezt;
	int i, j;
	
	for (j = 0; j < n_frames; j++)
		(*n_pose_arr)[j][0] = j;

	for (i = 0, ale = anim_data->first; ale; i++, ale = ale->next) {
		fcu = ale->key_data;

		for (j = 0, bezt = fcu->bezt; j < fcu->totvert; j++, bezt++)
			(*n_pose_arr)[j][i + 1] = bezt->vec[1][1];
	}
}

void ED_reduction_fill_pose_arr_fpoints(NPoseArr *n_pose_arr, ListBase *anim_data, int n_frames)
{
	bAnimListElem *ale;
	FCurve *fcu;
	FPoint *fpt;
	int i, j;

	for (j = 0; j < n_frames; j++)
		(*n_pose_arr)[j][0] = j;

	for (i = 0, ale = anim_data->first; ale; i++, ale = ale->next) {
		fcu = ale->key_data;

		for (j = 0, fpt = fcu->fpt; j < fcu->totvert; j++, fpt++)
			(*n_pose_arr)[j][i + 1] = fpt->vec[1];
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
 * composed of segments, where each segments is defined by a start frame, a finish frame, and a set of points. The cost
 * of the segment is taken to be the maximum perpendicular distance between the original f-curve and the closest chord
 * (a "chord" is the line between a neighboring pair of points).
 */

float ED_reduction_line_to_point_dist(float *p, float *q1, float *q2, const int npts)
{
	float q_p1[npts], q2_q1[npts], p_q1q2[npts];
	float t, numer, denom;

	sub_vn_vnvn(q_p1, p, q1, npts);
	sub_vn_vnvn(q2_q1, q2, q1, npts);

	numer = dot_vn_vn(q_p1, q2_q1, npts);
	denom = dot_vn_vn(q2_q1, q2_q1, npts);
	if (denom != 0)
		t = numer / denom;
	else
		t = numer;

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
 * A dynamic programming algorithm is used to try all possible placements of the desired number of keyframes. A
 * pose array is given, which is "learned" by the algorithm. This learning establishes two stoptables. The best 
 * placement of the keyframes can be read directly from the finished tables.
 *
 * A stoptable is the name we have given to the dynamic programming tables used by this algorithm. Each cell of a 
 * stoptable represents the most significant "stops" between two boundary "points", denoted by the row and column number
 * of the cell, respectively. Here the word point is used to denote a pose within the given pose array (see the pose 
 * array construction section), and a stop is a point that is placed between the boundary points. The end goal of the 
 * algorithm is to find the best n stops between the boundary points on the original motion curve.
 *
 * The first table the algorithm computes is called the zero-stop table (z_table). Here the term zero indicates that
 * each cell will feature the best zero stop path between the two points. The other table created is called the n-table,
 * where "n" suggests that each cell features the best n stop path between the two points. The n-table is created by 
 * duplicating the z-table first, and then is recursively updated where one stop is added during each update. Note that
 * the first time it is updated, the n-table will describe the best 1-stop path between each pair of points, or the best
 * most significant keyframe within the boundaries. After the second update the n-table will describe the best 2-stop 
 * path (2 keyframes between boundaries), the 3-stop path after the third update, and so on.
 *
 * The algorithm finishes when enough updates have been completed for the n-table to describe the most significant n-2
 * keyframes, where n is denoted by the n_keys variables which is provided by the operator. Note the -2 is because the 
 * first and last keyframes and given by the boundary points.
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

	for (i = 0; i < n_frames * n_frames; i++)
		MEM_freeN((*table)[i].path);
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

void ED_reduction_n_stoptable(int *indices, int n_frames, int n_keys, int n, StopTable n_table,
																			 StopTable n_tableBuffer,
																			 StopTable z_table)
{
	int i, j, k, ij, ik, kj;
	float cost_min, cost;

	if (n_table[n_frames - 1].n == n_keys) {
		ED_reduction_copy_indicies(indices, n_table[n_frames - 1].path, n_table[n_frames - 1].n);
		return;
	}

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
					ED_reduction_copy_indicies(n_tableBuffer[ij].path, n_table[ik].path, n_table[ik].n);
					n_tableBuffer[ij].path[n_table[ik].n] = j;
				}
			}
		}
	}

	ED_reduction_n_stoptable(indices, n_frames, n_keys, n + 1, n_tableBuffer, n_table, z_table);
}

/* Keyframe Data Caching -------------------------------------------------------------------------------------------- */
/*
 * These functions create, fill, and delete the frame cache data structure. The frame cache is used to remember the data
 * of an fcurve after it has been deleted.
 */

Frame *ED_reduction_init_frame_cache(int n)
{
	return MEM_mallocN(sizeof(Frame) * n, "FrameCache_new");
}

void ED_reduction_cache_fcurve_beztriples(Frame *cache, FCurve *fcu)
{
	Frame tmpFrame;
	BezTriple *bezt;
	int i;

	for (i = 0, bezt = fcu->bezt; i < fcu->totvert; i++, bezt++) {
		tmpFrame.f = bezt->vec[1][0];
		tmpFrame.v = bezt->vec[1][1];
		cache[i] = tmpFrame;
	}
}

void ED_reduction_cache_fcurve_fpoints(Frame *cache, FCurve *fcu)
{
	Frame tmpFrame;
	FPoint *fpt;
	int i;

	for (i = 0, fpt = fcu->fpt; i < fcu->totvert; i++, fpt++) {
		tmpFrame.f = fpt->vec[0];
		tmpFrame.v = fpt->vec[1];
		cache[i] = tmpFrame;
	}
}

void ED_reduction_cache_indices_of_fcurve_beztriples(Frame *cache, FCurve *fcu, int *indices, int n_keys)
{
	Frame tmpFrame;
	BezTriple *bezt;
	int i, index;

	index = 0;
	for (i = 0, bezt = fcu->bezt; i < fcu->totvert; i++, bezt++) {
		if (ED_reduction_val_in_array(i, indices, n_keys)) {
			tmpFrame.f = bezt->vec[1][0];
			tmpFrame.v = bezt->vec[1][1];
			cache[index] = tmpFrame;

			index ++;
		}
	}
}

void ED_reduction_cache_indices_of_fcurve_fpoints(Frame *cache, FCurve *fcu, int *indices, int n_keys)
{
	Frame tmpFrame;
	FPoint *fpt;
	int i, index;

	index = 0;
	for (i = 0, fpt = fcu->fpt; i < fcu->totvert; i++, fpt++) {
		if (ED_reduction_val_in_array(i, indices, n_keys)) {
			tmpFrame.f = fpt->vec[0];
			tmpFrame.v = fpt->vec[1];
			cache[index] = tmpFrame;
			index ++;
		}
	}
}


void ED_reduction_delete_frame_cache(Frame *cache)
{
	MEM_freeN(cache);	
}


/* Bezier Handle Tweaking ------------------------------------------------------------------------------------------- */
/*
 * This iterative algorithm tries a set of different bezier-handle placements for the new keyframes. If a placement 
 * that matches the original curve more closely is found, it is kept.
 *
 * This algorithm works by first posing the fcurve as a set of segments, where a segment is defined as any part of the 
 * curve that lies between two key-frames. For each segment, a number of different lengths for the inner bezier handles 
 * (right handle of keyframe to the left, and left handle of keyframe to the right). These lengths range from half the
 * width of the section below the keyframe, to half the width above. The number of tweaks is given by the
 * NUMBER_OF_TWEAKS variable.
 */

#define NUMBER_OF_TWEAKS 40

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

	half_segment_length = (end_f - start_f) / 2.0f;
	inc = half_segment_length / (NUMBER_OF_TWEAKS * 0.5f);
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


/* Reduction API ---------------------------------------------------------------------------------------------------- */
/*
 * Welcome! This module provides two algorithms used by the "Reduce Keyframes" wmOperator (GRAPH_OT_reduce). The module 
 * is still under-development, but should working relatively as intended. Please contact Riro (Richard Roberts) for any
 * questions, issues, or feedback - rykardo.r@gmail.com - thanks!
 *
 * These following functions are called directly by the operator. The first function runs the reduction algorithm to 
 * find the best n keyframes that represent the given animation, the second function reduces the keyframes of an fcurve
 * to those indicated by the given indices, and then third runs a bezier handle tweaking algorithm to manipulate an
 * fcurve in order to match the original data as closely as possible. 
 *
 * This module is broken into different sections, please refer to them for more information, or email me if those
 * comments are not sufficient - eventually I will document the code properly!
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
	ED_reduction_n_stoptable(indices, n_frames, n_keys, 0, n_table, n_tableBuffer, z_table);
	
	/* Clean up */
	ED_reduction_delete_stoptable(&z_table, n_frames);
	ED_reduction_delete_stoptable(&n_table, n_frames);
	ED_reduction_delete_stoptable(&n_tableBuffer, n_frames);
	ED_reduction_free_pose_arr(&n_pose_arr, n_frames);
}

void ED_reduction_reduce_fcurve_to_frames(FCurve *fcu, Frame *reduced_frames, int n_keys)
{
	int i;

	/* Free any existing data */
	if (fcu->bezt) MEM_freeN(fcu->bezt);
	if (fcu->fpt) MEM_freeN(fcu->fpt);
	fcu->bezt = NULL;
	fcu->fpt = NULL;
	fcu->totvert = 0;

	for (i = 0; i < n_keys; i++)
		insert_vert_fcurve(fcu, reduced_frames[i].f, reduced_frames[i].v, 1);
}

void ED_reduction_tweak_fcurve_anchors(FCurve *fcu, Frame *org_frames, Frame *reduced_frames)
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