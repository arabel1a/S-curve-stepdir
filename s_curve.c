/** Copyright (C) 2003 Mikhail Schirokikh
                       <github.com/arabel1a>
*/

/** This code is free software; you can redistribute it and/or
    modify it under the terms of version 3 of the GNU Library General
    Public License as published by the Free Software Foundation.
    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
    You should have received a copy of the GNU Library General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
    THE AUTHORS OF THIS LIBRARY ACCEPT ABSOLUTELY NO LIABILITY FOR
    ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
    TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
    harming persons must have provisions for completely removing power
    from all motors, etc, before persons enter any danger area.  All
    machinery must be designed to comply with local and national safety
    codes, and the author of this software can not, and do not, take
    any responsibility for such compliance.

*/



#include "s_curve.h"

const float eps = 0.001;    				// just a little thing to escape zero division in some calculations
const int newton_iterations = 10;


float _v1(float x, profile * accel){
     return accel->f_0 + accel->a * x * x;
 }

float _v2(float x, profile * accel){
     return accel->c*x + accel->d;
 }

float _v3(float x, profile * accel){
     return -accel->a*(x - accel->t_c) *(x - accel->t_c) + accel->f_c;
 }


float _n1 (float x, float k, profile * accel){
    return accel->f_0 * x + accel->a * x*x*x/3 - k;
}

float _n2 (float x, float k, profile * accel){
    return accel->c * x *x/2 + accel->d * x - accel->c * accel->t_1*accel->t_1/2 - accel->d * accel->t_1 + accel->N_1 - k;
}
float _n3 (float x, float k, profile * accel){
    return accel->f_c * x - accel->a * (x - accel->t_c)*(x - accel->t_c)*(x - accel->t_c)/3 - accel->f_c * (accel->t_c - accel->t_1) + accel->a * ( - accel->t_1 )*( - accel->t_1 )*( - accel->t_1 )/3  +  accel->N_2 - k;
}


void calculate_accel( profile * accel ){
    if( accel->f_c - accel->f_0 - accel->c*accel->t_c > 0)
        accel->t_c = (accel->f_c - accel->f_0) / accel->c + eps;
    if(accel->c > (2 * accel->f_c - 2*accel->f_0)  / accel->t_c)
        accel->c = (2 * accel->f_c - 2*accel->f_0)  / accel->t_c;
    accel->a =  -(accel->c*accel->c) / (2 * (accel->f_c - accel->f_0 - accel->c * accel->t_c));
    accel->d = accel->f_0 - accel->c *accel->c / (4*accel->a);
    accel->t_1 = accel->c / (2*accel->a);
    accel->N_1 = _n1(accel->t_1, 0, accel);
    accel->N_2 = _n2(accel->t_c - accel->t_1, 0, accel);
    accel->N_3 = _n3(accel->t_c, 0, accel);
    
    float ttim1[(int)accel->N_1];
    for(int j = 0; j <= accel->N_1; j++){
        ttim1[j] = 1/accel->f_0 + eps;
        for(int i = 0; i < newton_iterations; i++)
            ttim1[j] = ttim1[j] - _n1(ttim1[j], j, accel) / _v1(ttim1[j], accel) ;
        if(j > 0)
            accel->timings[j] = ttim1[j] - ttim1[j-1];
    }
    accel->timings[0] = 1/accel->f_0;

    float ttim2[(int)accel->N_2];
    for(int j = accel->N_1; j <= accel->N_2; j++){
        ttim2[j] = ttim1[(int)accel-> N_1 - 1];
        for(int i = 0; i < newton_iterations; i++)
            ttim2[j] = ttim2[j] - _n2(ttim2[j], j, accel) / _v2(ttim2[j], accel) ;
        if(j > accel->N_1)
            accel->timings[j] = ttim2[j] - ttim2[j-1];
    }
    

    float ttim3[(int)accel->N_3];
    for(int j = accel->N_2; j < accel->N_3; j++){
        ttim3[j] = ttim2[(int)accel-> N_2 - 1];
        for(int i = 0; i < newton_iterations; i++)
            ttim3[j] = ttim3[j] - _n3(ttim3[j], j, accel) / _v3(ttim3[j], accel) ;
        if(j > accel->N_2)
            accel->timings[j] = ttim3[j] - ttim3[j-1];
    }
}


void traj_planner(int curr_pos, int target_pos, profile * accel)
{
    if(accel->f_c <= accel->f_0)
        accel->f_0 = accel->f_c - 1;
    int _dir = target_pos > curr_pos ? 1 : -1;
    int path = _dir * ( target_pos - curr_pos);
    float tcl = 0;
    float tcr = accel->max_t_c - 1;
    while(tcr - tcl > 0.01){
        accel->t_c = (tcl + tcr) / 2;
        accel->a =  -(accel->c *accel->c) / (2 * (accel->f_c - accel->f_0 - accel->c * accel->t_c));
        accel->d = accel->f_0 - accel->c *accel->c / (4*accel->a);
        accel->t_1 = accel->c / (2*accel->a);
        accel->N_1 = _n1(accel->t_1, 0, accel);
        accel->N_2 = _n2(accel->t_c - accel->t_1, 0, accel);
        accel->N_3 = _n3(accel->t_c, 0, accel);
        if(accel->N_3 > path/2 - accel->ksi)
            tcr = accel->t_c;
        else
            tcl = accel->t_c;

    }

    if(accel->f_c - accel->f_0 - accel->c*accel->t_c > 0 || accel->N_1 < 3 || accel->N_3 - accel->N_2 < 3)
    {
        float f_cr = accel->c*accel->max_t_c + accel->f_0 - 10;
        float f_cl = 0;
        while(f_cr - f_cl > 1){
            accel->f_c = (f_cr + f_cl) / 2;
            tcl = 0;
            tcr = accel->max_t_c;
                while(tcr - tcl > 0.01){
                    accel->t_c = (tcl + tcr) / 2;
                    accel->a =  -(accel->c * accel->c) / (2 * (accel->f_c - accel->f_0 - accel->c * accel->t_c));
                    accel->d = accel->f_0 - accel->c * accel->c / (4*accel->a);
                    accel->t_1 = accel->c / (2*accel->a);
                    accel->N_1 = _n1(accel->t_1, 0, accel);
                    accel->N_2 = _n2(accel->t_c - accel->t_1, 0, accel);
                    accel->N_3 = _n3(accel->t_c, 0, accel);
                    if(accel->N_3 > path/2 - accel->ksi)
                        tcr = accel->t_c;
                    else
                        tcl = accel->t_c;
                }

            if(accel->f_c - accel->f_0 - accel->c*accel->t_c > 0 || accel->N_1 < 3 || accel->N_3 - accel->N_2 < 3)
                f_cr = accel->f_c;
            else
                f_cl = accel->f_c;
        }
    }
    calculate_accel(  accel );

}
