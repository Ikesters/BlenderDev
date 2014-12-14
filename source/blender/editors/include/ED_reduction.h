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

/** \file ED_reduction.h
 *  \ingroup editors
 */

#ifndef __ED_REDUCTION_H__
#define __ED_REDUCTION_H__


/* Utilities -------------------------------------------------------------------------------------------------------- */
/* 
 * Utility functions that are used in this module. I feel Blender must have these functions written somewhere that I am
 * not yet aware of, I intend to eventually remove them.
 */

bool   ED_reduction_val_in_array          (int val, int *arr, int size);
void   ED_reduction_substract_vectors     (int npts, double out[npts], double *a, double *b);
double ED_reduction_dot_vectors           (int npts, double *a, double *b);
void   ED_reduction_copy_vector           (int npts, double out[npts], double *a);
void   ED_reduction_scale_vector          (int npts, double out[npts], double s);
double ED_reduction_length_of_vector      (int npts, double * a);
int    ED_reduction_get_number_of_frames  (ListBase *anim_data);
int    ED_reduction_get_number_of_fcurves (ListBase *anim_data);


/* N-dimensional Curve Construction --------------------------------------------------------------------------------- */
/* 
 * Instead of running a cost function on n one-dimensional f-curves, we choose to use 1 n-dimensional curve. The
 * following functions create, fill, and delete this data structure.
 */

typedef double ** NCurve;

NCurve ED_reduction_alloc_ndim_curve (int n_frames, int n_curves);
void   ED_reduction_fill_ndim_curve  (NCurve ncurve, ListBase *anim_data, int n_frames);
void  ED_reduction_free_ndim_curve   (NCurve *ncurve);


/* Cost Analysis ---------------------------------------------------------------------------------------------------- */
/* 
 * The following functions are used when evaluating how successful a proposed reduction is. A proposed reduction is
 * composed of segments, where each segments is defined by a start frame, a finish frame, and a path. The cost of the
 * segment is taken to be the maximum perpendicular distance between the original f-curve and the multi-point path
 * (each pair of points in the path is referred to as a "chord").
 */

void   ED_reduction_substract_vectors   (int npts, double *out, double *a, double *b);
double ED_reduction_chord_to_frame_cost (double *p, double *q1, double *q2, int npts);
double ED_reduction_segment_cost        (NCurve ncurve, int start_f, int end_f, int n_curves);


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

typedef struct NStop {
	float cost;
	int n;
	int *path;
} NStop;

void ED_reduction_copy_stoptable_path         (int *tgt, int *src, int npts);
void ED_reduction_copy_stoptable_path_and_add (int *tgt, int *src, int npts, int v);
void ED_reduction_init_stoptable              (int npts_sq, NStop *table);
void ED_reduction_copy_stoptable              (int npts_sq, NStop *a, NStop *b);
void ED_reduction_delete_stoptable            (int npts_sq, NStop *table);
void ED_reduction_zero_stoptable              (int npts, int npts_sq, NStop *table, NCurve ncurve, int n_curves);
void ED_reduction_n_stoptable                 (int npts, int npts_sq, int n_stops, int n, NStop *n_table, NStop *z_table);


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

typedef struct Frame {
	double f;
	double v;
} Frame;

typedef struct Anchor {
	double p1;
	double p2;
} Anchor;

double ED_reduction_interpolation_at        (double f, double start_f, double end_f, Anchor anchors);
double ED_reduction_interpolation_cost      (Frame *org_frames, double start_f, double end_f, Anchor anchors);
Anchor ED_reduction_pick_anchor_for_segment (Frame *org_frames, double start_f, double end_f);
void   ED_reduction_tweak_fcurve_anchors    (Anchor *anchors, Frame *org_frames, Frame *reduced_frames, int n_reduced);


/* Reduction -------------------------------------------------------------------------------------------------------- */
/*
 * Welcome! This module provides two algorithms used by the "Reduce Keyframes" wmOperator (GRAPH_OT_reduce). The module 
 * is still under-development, but should working relatively as intended. Please contact Riro (Richard Roberts) for any
 * questions, issues, or feedback - rykardo.r@gmail.com - thanks!
 *
 * These functions are called directly by the operator. The first function runs the reduction algorithm to find the best
 * n keyframes that represented the given animation. The second function first reduces the keyframes of each motion
 * curve to those indicated by the given indices, and then runs the bezier handle tweaking algorithm.
 */

int *ED_reduction_pick_best_frames (ListBase *anim_data, int n_stops);
void ED_reduction_reduce_fcurves   (ListBase *anim_data, int *frameIndices, int n_stops);


/* Registration ----------------------------------------------------------------------------------------------------- */

void ED_operatortypes_reduction(void); 


#endif /* __ED_REDUCTION_H__ */

