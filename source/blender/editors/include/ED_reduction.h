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


/* Utilities ---------------------------------------------------------------- */

bool ED_reduction_is_in_array(int val, int *arr, int size);


/* F-Curve Roughness Anaylsis ----------------------------------------------- */

double ED_reduction_get_fcurve_roughness(struct FCurve *fcu);


/* Keyframe Placement Analysis ---------------------------------------------- */

struct Frame;

double ED_reduction_choord_to_frame_cost(struct Frame p, struct Frame q1, struct Frame q2);
double ED_reduction_path_cost(struct Frame *frames, int i, int j);


/* NStop Tables ------------------------------------------------------------- */

struct NStop;

void ED_reduction_copy_stoptable_path(int *tgt, int *src, int npts);
void ED_reduction_copy_stoptable_path_and_add(int *tgt, int *src, int npts, int v);
void ED_reduction_init_stoptable(int npts_sq, struct NStop *table);
void ED_reduction_copy_stoptable(int npts_sq, struct NStop *a, struct NStop *b);
void ED_reduction_delete_stoptable(int npts_sq, struct NStop *table);

void ED_reduction_zero_stoptable(int nPts, int npts_sq, struct NStop * table, struct Frame *frames);
void ED_reduction_n_stoptable(int npts, int npts_sq, int n_stops, int n, struct NStop *nTable, struct NStop *zTable);


/* Reduction ---------------------------------------------------------------- */

int *ED_reduction_pick_best_frames_fcurve(struct FCurve *fcu, int n_stops);
int *ED_reduction_pick_best_frames_fcurves(ListBase anim_data, int n_stops);
void ED_reduction_reduce_fcurve_to_frames(struct FCurve *fcu, int *frameIndicies, int n_stops);
void ED_reduction_reduce_fcurves_to_frames(ListBase anim_data, int *frameIndicies, int n_stops);

/* Registration, called in screen_ops.c:ED_operatortypes_screen() */
void ED_operatortypes_reduction(void); 

#endif /* __ED_REDUCTION_H__ */
