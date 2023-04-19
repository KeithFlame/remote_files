//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 13-Apr-2023 16:35:29
//

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

// Include Files
#include "main.h"
#include "forShooting_keith.h"
#include "forShooting_keith_terminate.h"
#include "odeCosserat_keith.h"
#include "rt_nonfinite.h"
#include "shootingOpt_keith.h"
#include "skewMatrix_keith.h"

// Function Declarations
static void argInit_12x1_real_T(double result[12]);

static void argInit_20x1_real_T(double result[20]);

static void argInit_3x1_real_T(double result[3]);

static void argInit_3x4_real_T(double result[12]);

static void argInit_4x1_real_T(double result[4]);

static void argInit_d20xd22_real_T(double result_data[], int result_size[2]);

static signed char argInit_int8_T();

static double argInit_real_T();

static void main_forShooting_keith();

static void main_odeCosserat_keith();

static void main_shootingOpt_keith();

static void main_skewMatrix_keith();

// Function Definitions
//
// Arguments    : double result[12]
// Return Type  : void
//
static void argInit_12x1_real_T(double result[12])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 12; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[20]
// Return Type  : void
//
static void argInit_20x1_real_T(double result[20])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 20; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[3]
// Return Type  : void
//
static void argInit_3x1_real_T(double result[3])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 3; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[12]
// Return Type  : void
//
static void argInit_3x4_real_T(double result[12])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 3; idx0++) {
    for (int idx1{0}; idx1 < 4; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + 3 * idx1] = argInit_real_T();
    }
  }
}

//
// Arguments    : double result[4]
// Return Type  : void
//
static void argInit_4x1_real_T(double result[4])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 4; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result_data[]
//                int result_size[2]
// Return Type  : void
//
static void argInit_d20xd22_real_T(double result_data[], int result_size[2])
{
  // Set the size of the array.
  // Change this size to the value that the application requires.
  result_size[0] = 2;
  result_size[1] = 2;
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 2; idx0++) {
    for (int idx1{0}; idx1 < 2; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result_data[idx0 + 2 * idx1] = argInit_real_T();
    }
  }
}

//
// Arguments    : void
// Return Type  : signed char
//
static signed char argInit_int8_T()
{
  return 0;
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_forShooting_keith()
{
  double b_y1[2706];
  double y2[860];
  double y3[779];
  double t1_data[123];
  double b_y0[66];
  double t2_data[43];
  double t3_data[41];
  double Rsd[20];
  double b_dv[20];
  double qa_tmp[12];
  double t0[3];
  int t1_size;
  int t2_size;
  int t3_size;
  // Initialize function 'forShooting_keith' input arguments.
  // Initialize function input argument 'Guess'.
  // Initialize function input argument 'qa'.
  argInit_12x1_real_T(qa_tmp);
  // Initialize function input argument 'ksi'.
  // Call the entry-point 'forShooting_keith'.
  argInit_20x1_real_T(b_dv);
  forShooting_keith(b_dv, qa_tmp, qa_tmp, Rsd, t0, b_y0, t1_data,
                    *(int(*)[1]) & t1_size, b_y1, t2_data,
                    *(int(*)[1]) & t2_size, y2, t3_data, *(int(*)[1]) & t3_size,
                    y3);
}

//
// Arguments    : void
// Return Type  : void
//
static void main_odeCosserat_keith()
{
  double y_data[1320];
  double y0_data[440];
  double U_data[180];
  double t_data[60];
  double b_dv[12];
  double b_dv1[4];
  int U_size[2];
  int t_size[2];
  int y0_size[2];
  int y_size[2];
  // Initialize function 'odeCosserat_keith' input arguments.
  // Initialize function input argument 'y0'.
  argInit_d20xd22_real_T(y0_data, y0_size);
  // Initialize function input argument 'v'.
  // Initialize function input argument 'ksi'.
  // Call the entry-point 'odeCosserat_keith'.
  argInit_3x4_real_T(b_dv);
  argInit_4x1_real_T(b_dv1);
  odeCosserat_keith(y0_data, y0_size, b_dv, argInit_int8_T(), b_dv1, t_data,
                    t_size, y_data, y_size, U_data, U_size);
}

//
// Arguments    : void
// Return Type  : void
//
static void main_shootingOpt_keith()
{
  double b_y1[2706];
  double y2[860];
  double y3[779];
  double t1_data[123];
  double b_y0[66];
  double t2_data[43];
  double t3_data[41];
  double Guess[20];
  double qa_tmp[12];
  double t0[3];
  int t1_size;
  int t2_size;
  int t3_size;
  // Initialize function 'shootingOpt_keith' input arguments.
  // Initialize function input argument 'Guess'.
  // Initialize function input argument 'qa'.
  argInit_12x1_real_T(qa_tmp);
  // Initialize function input argument 'ksi'.
  // Call the entry-point 'shootingOpt_keith'.
  argInit_20x1_real_T(Guess);
  shootingOpt_keith(Guess, qa_tmp, qa_tmp, t0, t1_data, *(int(*)[1]) & t1_size,
                    t2_data, *(int(*)[1]) & t2_size, t3_data,
                    *(int(*)[1]) & t3_size, b_y0, b_y1, y2, y3);
}

//
// Arguments    : void
// Return Type  : void
//
static void main_skewMatrix_keith()
{
  double T[9];
  double b_dv[3];
  // Initialize function 'skewMatrix_keith' input arguments.
  // Initialize function input argument 'p'.
  // Call the entry-point 'skewMatrix_keith'.
  argInit_3x1_real_T(b_dv);
  skewMatrix_keith(b_dv, T);
}

//
// Arguments    : int argc
//                char **argv
// Return Type  : int
//
int main(int, char **)
{
  // The initialize function is being called automatically from your entry-point
  // function. So, a call to initialize is not included here. Invoke the
  // entry-point functions.
  // You can call entry-point functions multiple times.
  main_forShooting_keith();
  main_odeCosserat_keith();
  main_shootingOpt_keith();
  main_skewMatrix_keith();
  // Terminate the application.
  // You do not need to do this more than one time.
  forShooting_keith_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
