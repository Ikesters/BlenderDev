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
/* 
 * Utility functions that are used in this module. I feel Blender must have these functions written somewhere that I am
 * not yet aware of, I intend to eventually remove them.
 */

bool ED_reduction_val_in_array(int val, int *arr, int size)
{
	for (int i = 0; i < size; i++) {
		if (arr[i] == val)
			return true;
	}
	return false;
}

void ED_reduction_substract_vectors(int npts, double out[npts], double *a, double *b)
{
	for (int i = 0; i < npts; i++)
		out[i] = a[i] - b[i];
}

double ED_reduction_dot_vectors(int npts, double *a, double *b)
{
	double out = 0;
	for (int i = 0; i < npts; i++)
		out += a[i] * b[i];

	return out;
}

void ED_reduction_copy_vector(int npts, double out[npts], double *a)
{
	for (int i = 0; i < npts; i++)
		out[i] = a[i];
}

void ED_reduction_scale_vector(int npts, double out[npts], double s)
{
	for (int i = 0; i < npts; i++)
		out[i] = out[i] * s;
}

double ED_reduction_length_of_vector(int npts, double * a)
{
	double summedSqaures = 0;
	for (int i = 0; i < npts; i++)
		summedSqaures += pow(a[i], 2);

	return sqrt(summedSqaures);
}

int ED_reduction_get_number_of_frames(ListBase anim_data)
{
	for (bAnimListElem *ale = anim_data.first; ale; ale = ale->next) {
		FCurve * fcu = (FCurve *)ale->key_data;
		return fcu->totvert;
	}

	return -1;
}

int ED_reduction_get_number_of_fcurves(ListBase anim_data)
{
	int n = 0;
	for (bAnimListElem *ale = anim_data.first; ale; ale = ale->next)
		n++;

	return n;
}


/* N-dimensional Curve Construction --------------------------------------------------------------------------------- */
/* 
 * Instead of running a cost function on n one-dimensional f-curves, we choose to use 1 n-dimensional curve. The
 * following functions create, fill, and delete this data structure.
 */

NCurve ED_reduction_alloc_ndim_curve(int n_frames, int n_curves)
{
	NCurve ncurve = malloc (n_frames * sizeof (double *));
	for (int i = 0; i < n_frames; i++) {
		ncurve[i] = malloc (n_curves * sizeof (double));
	}

	return ncurve;
}

void ED_reduction_free_ndim_curve(NCurve *ncurve) {
	free(ncurve);
}

void ED_reduction_fill_ndim_curve(NCurve ncurve, ListBase anim_data, int n_frames)
{ 
	bAnimListElem *ale;
	BezTriple *bezt;
	int i, j;
	
	for (j = 0; j < n_frames; j++)
		ncurve[j][0] = (double) j;

	for (i = 0, ale = anim_data.first; ale; i++, ale = ale->next) {
		FCurve * fcu = (FCurve *)ale->key_data;

		for (j = 0, bezt = fcu->bezt; j < fcu->totvert; j++, bezt++)
			ncurve[j][i + 1] = bezt->vec[1][1];
	}
}


/* Cost Analysis ---------------------------------------------------------------------------------------------------- */
/* 
 * The following functions are used when evaluating how successful a proposed reduction is. A proposed reduction is
 * composed of segments, where each segments is defined by a start frame, a finish frame, and a path. The cost of the
 * segment is taken to be the maximum perpendicular distance between the original f-curve and the multi-point path
 * (each pair of points in the path is referred to as a "chord").
 */

double ED_reduction_chord_to_frame_cost(double *p, double *q1, double *q2, int npts)
{
	double distPoint[npts];
	double distLine[npts]; 
	double maxDist[npts];
	double t;

	ED_reduction_substract_vectors(npts, distPoint, p, q1);
	ED_reduction_substract_vectors(npts, distLine, q2, q1);

	t = ED_reduction_dot_vectors(npts, distPoint, distLine) / ED_reduction_dot_vectors(npts, distLine, distLine);
	ED_reduction_scale_vector(npts, distLine, t);
	ED_reduction_substract_vectors(npts, maxDist, distPoint, distLine);

	return ED_reduction_length_of_vector(npts, maxDist);
}

double ED_reduction_segment_cost(NCurve ncurve, int start_f, int end_f, int n_curves)
{
	double maxDist = 0;

	for (int i = start_f; i < end_f; i++) {
		double dist = ED_reduction_chord_to_frame_cost(ncurve[i], ncurve[start_f], ncurve[end_f], n_curves);
		
		if (dist > maxDist)
			maxDist = dist;
	}

	return maxDist;
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

void ED_reduction_zero_stoptable(int npts, int npts_sq, NStop table[npts_sq], NCurve ncurve, int n_curves)
{
	for (int i = 0; i < npts - 1; i++) {
		for (int j = i + 1; j < npts; j++) {
			
			int index = i * npts + j;
			table[index].cost = ED_reduction_segment_cost(ncurve, i, j, n_curves);
			table[index].n = 2;

			table[index].path = malloc(table[index].n * sizeof(int));  
			table[index].path[0] = i;
			table[index].path[1] = j;
		}
	}
}

void ED_reduction_n_stoptable(int npts, int npts_sq, int n_stops, int n, NStop n_table[npts_sq], NStop z_table[npts_sq])
{
	if (n_table[npts - 1].n == n_stops)
		return;

	NStop tmp_table[npts_sq];
	ED_reduction_init_stoptable(npts_sq, tmp_table);

	for (int i = 0; i < npts; i++) {
		for (int j = i + n + 1; j < npts; j++) {
			int indexIJ = i * npts + j;

			/* Find the least cost path between i and j */
			float minCost = 99999;
			for (int k = i + 1; k < j; k++) {
				int indexIK = i * npts + k;
				int indexKJ = k * npts + j;
				float cost = max_ff(n_table[indexIK].cost, z_table[indexKJ].cost);

				if (cost < minCost) {
					minCost = cost;	
					tmp_table[indexIJ].cost = cost;
					tmp_table[indexIJ].n = n_table[indexIK].n + 1;
					tmp_table[indexIJ].path = malloc(tmp_table[indexIJ].n * sizeof(int));  
					ED_reduction_copy_stoptable_path_and_add(tmp_table[indexIJ].path,
															 n_table[indexIK].path,
															 n_table[indexIK].n, j);
				}
			}
		}
	}

	ED_reduction_delete_stoptable(npts_sq, n_table);
	ED_reduction_copy_stoptable(npts_sq, tmp_table, n_table);
	ED_reduction_n_stoptable(npts, npts_sq, n_stops, n + 1, n_table, z_table);
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

double ED_reduction_interpolation_at(double f, double start_f, double end_f, Anchor anchors)
{
	double numer = f - start_f;
	double denom = end_f - start_f;
	double t = denom != 0.0 ? numer / denom : numer;

	return                 pow(1 - t, 3) * start_f     + 
	       3 *     t     * pow(1 - t, 2) * anchors.p1 +
           3 * pow(t, 2) *    (1 - t   ) * anchors.p2 +
               pow(t ,3)                 * end_f;
}

double ED_reduction_interpolation_cost(Frame *original_frames, double start_f, double end_f, Anchor anchors) {
	double maxCost = 0;

	int index = 0;
	for (double i = start_f; i < end_f; i += 1.0) {
		double originalV = original_frames[index].v;
		double interpedV = ED_reduction_interpolation_at(i, start_f, end_f, anchors);

		double cost = fabs(originalV - interpedV);
		if (cost > maxCost)
			maxCost = cost;

		index ++;
	}

	return maxCost;
}

Anchor ED_reduction_pick_anchor_for_segment(Frame *original_frames, double start_f, double end_f) {
	double half_segment_length = (end_f - start_f) / 2.0;
	double inc = half_segment_length / 20.0;

	double minCost = 99999.0;
	double bestI = 0.0;
	double bestJ = 0.0;

	for (double i = -half_segment_length; i < half_segment_length; i += inc) {
		for (double j = -half_segment_length; j < half_segment_length; j += inc) {

			double cost = ED_reduction_interpolation_cost(original_frames, start_f, end_f, (Anchor) { i, j });
			if (cost < minCost) {
				minCost = cost;
				bestI = i;
				bestJ = j;
			}
		}
	}

	return (Anchor) { bestI, bestJ };
}

void ED_reduction_tweak_fcurve_anchors(Anchor *anchors, Frame *org_frames, Frame *reduced_frames, int n_reduced)
{
	int i;
	for (i = 0; i < n_reduced; i++)
		anchors[i] = (Anchor) { reduced_frames[i].f, reduced_frames[i].v };

	for (i = 1; i < n_reduced; i++) {
		double start_f = reduced_frames[i - 1].f;
		double end_f = reduced_frames[i].f;

		Anchor anchor = ED_reduction_pick_anchor_for_segment(original_frames, start_f, end_f);
		anchors[i - 1].p2 = anchor.p1;
		anchors[i    ].p1 = anchor.p2;
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
 
int *ED_reduction_pick_best_frames(ListBase anim_data, int n_stops)
{
	int n_curves = ED_reduction_get_number_of_fcurves(anim_data) + 1;
	int n_frames = ED_reduction_get_number_of_frames(anim_data);
	int n_frames_sq = n_frames * n_frames;

	/* Construct n-dimensional curve */
	NCurve ncurve = ED_reduction_alloc_ndim_curve(n_frames, n_curves);
	ED_reduction_fill_ndim_curve(ncurve, anim_data, n_frames);

	/* Build dynamic-programming tables */
	NStop z_table[n_frames_sq];
	NStop n_table[n_frames_sq];
	ED_reduction_init_stoptable(n_frames_sq, z_table);
	ED_reduction_init_stoptable(n_frames_sq, n_table);
	ED_reduction_zero_stoptable(n_frames, n_frames_sq, z_table, ncurve, n_curves);
	ED_reduction_copy_stoptable(n_frames_sq, z_table, n_table);

	/* Recursively find the best point-path the first and last frame. */
	ED_reduction_n_stoptable(n_frames, n_frames_sq, n_stops, 0, n_table, z_table);

	/* Store frame indicies */
	int *frameIndices = malloc(n_table[n_frames - 1].n * sizeof (int));
	ED_reduction_copy_stoptable_path(frameIndices, n_table[n_frames - 1].path, n_table[n_frames - 1].n);
	
	/* Clean up */
	ED_reduction_delete_stoptable(n_frames_sq, z_table);
	ED_reduction_delete_stoptable(n_frames_sq, n_table);
	ED_reduction_free_ndim_curve(&ncurve);
	
	return frameIndices;
}

void ED_reduction_reduce_fcurves(ListBase anim_data, int *frameIndices, int n_stops)
{
	bAnimListElem *ale;
	BezTriple *bezt;
	FCurve * fcu;
	int i, n_frames;

	for (ale = anim_data.first; ale; ale = ale->next) {
		fcu = (FCurve *)ale->key_data;
		n_frames = fcu->totvert;
		
		/* Cache frame information */
		Frame original_frames[n_frames];
		Frame reduced_frames[n_stops];
		int index = 0;
		for (i = 0, bezt = fcu->bezt; i < n_frames; i++, bezt++) {
			original_frames[i] = (Frame) { bezt->vec[1][0], bezt->vec[1][1] };
			if (ED_reduction_val_in_array(i, frameIndices, n_stops)) {
				reduced_frames[index] = (Frame) { bezt->vec[1][0], bezt->vec[1][1] };
				index ++;
			}
		}

		/* Delete all the existing keyframes, and then rebuild based on the cached keyframe data */
		clear_fcurve_keys(fcu);
		for (i = 0; i < n_stops; i++)
			insert_vert_fcurve(fcu, reduced_frames[i].f, reduced_frames[i].v, 1);
		calchandles_fcurve(fcu);

		/* Tweaking the bezier handles of the new keyframes to best repliciate the original data */
		Anchor anchors[n_stops];
		ED_reduction_pick_anchors_for_fcurve(anchors, original_frames, reduced_frames, n_stops);
		for (i = 0, bezt = fcu->bezt; i < fcu->totvert; i++, bezt++) {
			if (i != 0)
				bezt->vec[0][1] = anchors[i].p1;
			if (i != n_stops - 1)
				bezt->vec[2][1] = anchors[i].p2;
		}

		ale->update |= ANIM_UPDATE_DEFAULT;
	}
}

