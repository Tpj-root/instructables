/********************************************************************
 * Description: a_table_b_head_kins.c
 *  Based on example of maxkins. Derived for Minitech desktop mill with rotary a-axis
 *  on table and b-axis on the head. Draft
 *
 * Author: Danie Gouws
 * License: GPL Version 2
 *
 * Copyright (c) 2007 Chris Radek
 ********************************************************************/

/********************************************************************
 * Note: The direction of the B axis is the opposite of the
 * conventional axis direction. See
 * https://linuxcnc.org/docs/html/gcode/machining-center.html
 ********************************************************************/

#include "kinematics.h" /* these decls */
#include "posemath.h"
#include "hal.h"
#include "rtapi.h"
#include "rtapi_math.h"

#define d2r(d) ((d)*PM_PI / 180.0)
#define r2d(r) ((r)*180.0 / PM_PI)

#ifndef hypot
#define hypot(a, b) (sqrt((a) * (a) + (b) * (b)))
#endif

struct haldata
{
    hal_float_t *pivot_length;
    hal_float_t *y_pos;             //position of rotary axis in machine coordinates
    hal_float_t *z_pos;
    hal_float_t *tool_length;
    

} *haldata;

int kinematicsForward(const double *joints,
                      EmcPose *pos,
                      const KINEMATICS_FORWARD_FLAGS *fflags,
                      KINEMATICS_INVERSE_FLAGS *iflags)
{
   
    double x1 = 0, y1 = 0, z1 = 0; // can probably be left out using 'touch-off'
    double x2 = joints[0];
    double y2 = joints[1] - *(haldata->y_pos);
    double z2 = joints[2] - *(haldata->z_pos) + *(haldata->pivot_length);       // distance between pivot points, change assignment for easier setup
    double z3 = -1* *(haldata->pivot_length) - *(haldata->tool_length); // head+ tool length

    double a = -1*d2r(joints[3]);
    double b = d2r(joints[4]);

    pos->tran.x = -sin(b) * z3 + x2 + x1;
    pos->tran.y = sin(a) * cos(b) * z3 + cos(a) * y2 + sin(a) * z2 + y1;
    pos->tran.z = cos(a) * cos(b) * z3 - sin(a) * y2 + cos(a) * z2 + z1;
    //pos->tran.z = cos(a) * cos(b) * z3 - sin(a) * x2 + cos(a) * z2 + z1;
    //pos->tran.z = cos(a) * cos(b) * z3  + cos(a) * z2 + z1;
    pos->a = joints[3];
    pos->b = joints[4];

    pos->c = joints[5];
    pos->u = joints[6];
    pos->v = joints[7];
    pos->w = joints[8];

    return 0;
}

int kinematicsInverse(const EmcPose *pos,
                      double *joints,
                      const KINEMATICS_INVERSE_FLAGS *iflags,
                      KINEMATICS_FORWARD_FLAGS *fflags)
{
 
    double Qx = pos->tran.x;
    double Qy = pos->tran.y;
    double Qz = pos->tran.z;
    double a = -1*d2r(pos->a);
    double b = d2r(pos->b);

    double x1 = 0, y1 = 0, z1 = 0;
    double z3 = -1* *(haldata->pivot_length)- *(haldata->tool_length);

    double x2 = sin(b) * z3 + Qx - x1;
    double y2 = cos(a) * Qy - sin(a) * Qz + sin(a) * z1 - cos(a) * y1;
    double z2 = -cos(b) * z3 + sin(a) * Qy + cos(a) * Qz - sin(a) * x1 - cos(a) * z1;

    joints[0] = x2;
    joints[1] = y2 + *(haldata->y_pos);
    //joints[2] = z2 + *(haldata->z_pos) - *(haldata->pivot_length);   
    joints[2] = z2 + *(haldata->z_pos) - *(haldata->pivot_length); 

    joints[3] = pos->a;
    joints[4] = pos->b;

    joints[5] = pos->c;
    joints[6] = pos->u;
    joints[7] = pos->v;
    joints[8] = pos->w;

    return 0;
}

KINEMATICS_TYPE kinematicsType()
{
    return KINEMATICS_BOTH;
    //return KINEMATICS_INVERSE_ONLY;
}


#include "rtapi.h"     /* RTAPI realtime OS API */
#include "rtapi_app.h" /* RTAPI realtime module decls */

//KINS_NOT_SWITCHABLE //gives a error
EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsInverse);
EXPORT_SYMBOL(kinematicsForward);
MODULE_LICENSE("GPL");

int comp_id;
int rtapi_app_main(void)
{
    int result;
    comp_id = hal_init("a_table_b_head_kins");
    if (comp_id < 0)
        return comp_id;

    haldata = hal_malloc(sizeof(struct haldata));

    result = hal_pin_float_new("a_table_b_head_kins.pivot-length", HAL_IO, &(haldata->pivot_length), comp_id);
    result += hal_pin_float_new("a_table_b_head_kins.y_pos", HAL_IO, &(haldata->y_pos), comp_id);
    result += hal_pin_float_new("a_table_b_head_kins.z_pos", HAL_IO, &(haldata->z_pos), comp_id);
    result += hal_pin_float_new("a_table_b_head_kins.tool_length", HAL_IN, &(haldata->tool_length), comp_id);
    if (result < 0)
        goto error;

    *(haldata->pivot_length) = 60.3756;
    //94.3;//119.4;
    *(haldata->y_pos) = 76.5552;
    *(haldata->z_pos) = -71.0565;
    //-36.505;   //machine ccord when vertical tip in line with axis
    //*(haldata->tool_length) = 24.123;

    hal_ready(comp_id);
    return 0;

error:
    hal_exit(comp_id);
    return result;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }
