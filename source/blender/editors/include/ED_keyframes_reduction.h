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
 * Various utility functions, nothing interesting to see here.
 */

void   ED_reduction_copy_indicies         (int *tgt, int *src, int npts);
bool   ED_reduction_val_in_array          (int val, int *arr, int size);
int    ED_reduction_get_number_of_frames  (ListBase *anim_data);


/* Pose Array Construction ------------------------------------------------------------------------------------------ */
/* 
 * Instead of running a cost function on n one-dimensional fcurves, we choose to use 1 n-dimensional curve. The
 * following functions create, fill, and delete this data structure. This n-dimensional curve can be imagined as an
 * array of poses, where the component of that array at index i contains the pose for frame i. The pose itself is an 
 * of the value of each fcurve at frame i.
 */

typedef float **NPoseArr;

void ED_reduction_init_pose_arr               (NPoseArr *n_pose_arr, int n_frames, int n_curves);
void ED_reduction_fill_pose_arr_beziertriples (NPoseArr *n_pose_arr, ListBase *anim_data, int n_frames);
void ED_reduction_fill_pose_arr_fpoints       (NPoseArr *n_pose_arr, ListBase *anim_data, int n_frames);
void ED_reduction_free_pose_arr               (NPoseArr *n_pose_arr, int n_frames);


/* Cost Analysis ---------------------------------------------------------------------------------------------------- */
/* 
 * The following functions are used when evaluating how successful a proposed reduction is. A proposed reduction is
 * composed of segments, where each segments is defined by a start frame, a finish frame, and a set of points. The cost
 * of the segment is taken to be the maximum perpendicular distance between the original f-curve and the closest chord
 * (a "chord" is the line between a neighboring pair of points).
 */

float ED_reduction_line_to_point_dist (float *p, float *q1, float *q2, const int npts);
float ED_reduction_segment_cost       (NPoseArr *n_pose_arr, int start_f, int end_f, int n_curves);


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

typedef struct NStop {
	float cost;
	int n;
	int *path;
} NStop;
typedef NStop *StopTable;

void ED_reduction_init_stoptable   (StopTable *table, int n_frames, int n_keys);
void ED_reduction_copy_stoptable   (StopTable a, StopTable b, int n_frames);
void ED_reduction_delete_stoptable (StopTable *table, int n_frames);
void ED_reduction_zero_stoptable   (StopTable table, NPoseArr *n_pose_arr, int n_frames, int n_curves);
void ED_reduction_n_stoptable      (int *indices, int n_frames, int n_keys, int n, StopTable n_table,
																				   StopTable n_tableBuffer,
																				   StopTable z_table);


/* Keyframe Data Caching -------------------------------------------------------------------------------------------- */
/*
 * These functions create, fill, and delete the frame cache data structure. The frame cache is used to remember the data
 * of an fcurve after it has been deleted.
 */

typedef struct Frame {
	float f;
	float v;
} Frame;

Frame *ED_reduction_init_frame_cache                   (int n);
void   ED_reduction_cache_fcurve_beztriples            (Frame *cache, FCurve *fcu);
void   ED_reduction_cache_fcurve_fpoints               (Frame *cache, FCurve *fcu);
void   ED_reduction_cache_indices_of_fcurve_beztriples (Frame *cache, FCurve *fcu, int *indices, int n_keys);
void   ED_reduction_cache_indices_of_fcurve_fpoints    (Frame *cache, FCurve *fcu, int *indices, int n_keys);
void   ED_reduction_delete_frame_cache                 (Frame *cache);


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

typedef struct Anchor {
	float p1;
	float p2;
} Anchor;

float  ED_reduction_interpolation_at       (float f, float start_f, float end_f, Anchor anchors);
float  ED_reduction_interpolation_cost     (Frame *org_frames, float start_f, float end_f, Anchor anchors);
Anchor ED_reduction_pick_anchor_for_segment(Frame *org_frames, float start_f, float end_f);


/* Reduction -------------------------------------------------------------------------------------------------------- */
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

void ED_reduction_pick_best_frames        (NPoseArr n_pose_arr, int n_keys, int n_frames, int n_curves, int *indices);
void ED_reduction_reduce_fcurve_to_frames (FCurve *fcu, Frame *reduced_frames, int n_keys);
void ED_reduction_tweak_fcurve_anchors    (FCurve *fcu, Frame *org_frames, Frame *reduced_frames);


/* Registration ----------------------------------------------------------------------------------------------------- */

void ED_operatortypes_reduction(void); 


#endif /* __ED_REDUCTION_H__ */

