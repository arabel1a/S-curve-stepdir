/*
*    Copyright (C) 2003 Mikhail Schirokikh
*                       <github.com/arabel1a>
*
*
*    This code is free software; you can redistribute it and/or
*    modify it under the terms of version 3 of the GNU Library General
*    Public License as published by the Free Software Foundation.
*    This library is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU Lesser General Public License for more details.
*    You should have received a copy of the GNU Library General Public
*    License along with this library; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*    THE AUTHORS OF THIS LIBRARY ACCEPT ABSOLUTELY NO LIABILITY FOR
*    ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
*    TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
*    harming persons must have provisions for completely removing power
*    from all motors, etc, before persons enter any danger area.  All
*    machinery must be designed to comply with local and national safety
*    codes, and the author of this software can not, and do not, take
*    any responsibility for such compliance.
*/


struct profile{
    float f_0;                      // starting speed
    float f_c;                      // target speed
    float t_c;                      // target accel time
    float max_t_c;					// maximum time for acceleration
    float c;                        // maximum acceleration
    float a;                        // 1st parabola coefficient 
    float d;                        // linear displacement coefficient
    float t_1;                      // switching time (from 1st parabola to linear part, second switch point is symmetric)
    float N_1;                      // last pulse at 1st stage
    float N_2;                      // last pulse at 2nd stage
    float N_3;                      // last pulse at 3nd stage
    float * timings;                // pointer to table of timings
	int ksi;             			// steps away of center to be gone on target speed
};
typedef struct profile profile;

/*
*   See the descriptions of thoose in file main.c
*/
float _v1(float x, profile * accel);

float _v2(float x, profile * accel);
 

float _v3(float x, profile * accel);


float _n1 (float x, float k, profile * accel);

float _n2 (float x, float k, profile * accel);

float _n3 (float x, float k, profile * accel);

void calculate_accel( profile * accel );


void traj_planner(int curr_pos, int target_pos, profile * accel);

