/****************************************************************************
 *
 *  DHD - Haptic SDK ver 3.7.0-pre
 *  (C) 2001-2016 Force Dimension
 *  All Rights Reserved.
 *
 *  contact: support@forcedimension.com
 *
 ****************************************************************************/


/* C header */

#ifndef __DHDC_H__
#define __DHDC_H__

#include <cstddef>


/****************************************************************************
 *  GLOBAL DEFINITIONS
 ****************************************************************************/

#if (defined(WIN32) | defined(WIN64)) & !defined(WINCE7)
#define __SDK __stdcall
#else
#define __SDK
#endif

#ifdef __cplusplus
extern "C" {
#endif


/****************************************************************************
 *  TYPES
 ****************************************************************************/

typedef unsigned char  uchar;
typedef unsigned short ushort;
typedef unsigned int   uint;
typedef unsigned long  ulong;


/****************************************************************************
 *  ERROR MANAGEMENT
 ****************************************************************************/

  /* error codes */
  enum dhd_errors {
    DHD_NO_ERROR,
    DHD_ERROR,
    DHD_ERROR_COM,
    DHD_ERROR_DHC_BUSY,
    DHD_ERROR_NO_DRIVER_FOUND,
    DHD_ERROR_NO_DEVICE_FOUND,
    DHD_ERROR_NOT_AVAILABLE,
    DHD_ERROR_TIMEOUT,
    DHD_ERROR_GEOMETRY,
    DHD_ERROR_EXPERT_MODE_DISABLED,
    DHD_ERROR_NOT_IMPLEMENTED,
    DHD_ERROR_OUT_OF_MEMORY,
    DHD_ERROR_DEVICE_NOT_READY,
    DHD_ERROR_FILE_NOT_FOUND,
    DHD_ERROR_CONFIGURATION,
    DHD_ERROR_INVALID_INDEX,
    DHD_ERROR_DEPRECATED,
    DHD_ERROR_NULL_ARGUMENT,
    DHD_ERROR_REDUNDANT_FAIL,
    DHD_ERROR_NOT_ENABLED,
    DHD_ERROR_DEVICE_IN_USE,
    DHD_ERROR_INVALID
  };

  /* error reporting */
  int         __SDK dhdErrorGetLast    ();
  const char* __SDK dhdErrorGetLastStr ();
  const char* __SDK dhdErrorGetStr     (int error);


/****************************************************************************
 *  CONSTANTS
 ****************************************************************************/

/* devices */
#define DHD_DEVICE_NONE             0
#define DHD_DEVICE_DELTA3          63
#define DHD_DEVICE_DELTA6          64
#define DHD_DEVICE_OMEGA3          33
#define DHD_DEVICE_OMEGA33         34
#define DHD_DEVICE_OMEGA33_LEFT    36
#define DHD_DEVICE_OMEGA331        35
#define DHD_DEVICE_OMEGA331_LEFT   37
#define DHD_DEVICE_FALCON          60
#define DHD_DEVICE_CONTROLLER      81
#define DHD_DEVICE_CONTROLLER_HR   82
#define DHD_DEVICE_CUSTOM          91
#define DHD_DEVICE_SIGMA331       104
#define DHD_DEVICE_SIGMA331_LEFT  105
#define DHD_DEVICE_SIGMA33P       106
#define DHD_DEVICE_SIGMA33P_LEFT  107

/* deprecated devices (no longer supported) */
#define DHD_DEVICE_3DOF            31
#define DHD_DEVICE_6DOF            61
#define DHD_DEVICE_6DOF_500        62
#define DHD_DEVICE_OMEGA           32

/* status */
#define DHD_ON                      1
#define DHD_OFF                     0
#define DHD_UNDEFINED              -1

/* encoder count */
#define DHD_MAX_DOF                 8

/* delta motor index */
#define DHD_DELTA_MOTOR_0           0
#define DHD_DELTA_MOTOR_1           1
#define DHD_DELTA_MOTOR_2           2

/* delta encoder index */
#define DHD_DELTA_ENC_0             0
#define DHD_DELTA_ENC_1             1
#define DHD_DELTA_ENC_2             2

/* wrist motor index */
#define DHD_WRIST_MOTOR_0           3
#define DHD_WRIST_MOTOR_1           4
#define DHD_WRIST_MOTOR_2           5

/* wrist encoder index */
#define DHD_WRIST_ENC_0             3
#define DHD_WRIST_ENC_1             4
#define DHD_WRIST_ENC_2             5

/* gripper encoder index */
#define DHD_GRIP_ENC                6
#define DHD_GRIP_MOT                6

/* useful non-error, positive return values */
#define DHD_TIMEGUARD               1
#define DHD_MOTOR_SATURATED         2

/* status count */
#define DHD_MAX_STATUS             16

/* status codes */
#define DHD_STATUS_POWER            0
#define DHD_STATUS_CONNECTED        1
#define DHD_STATUS_STARTED          2
#define DHD_STATUS_RESET            3
#define DHD_STATUS_IDLE             4
#define DHD_STATUS_FORCE            5
#define DHD_STATUS_BRAKE            6
#define DHD_STATUS_TORQUE           7
#define DHD_STATUS_WRIST_DETECTED   8
#define DHD_STATUS_ERROR            9
#define DHD_STATUS_GRAVITY         10
#define DHD_STATUS_TIMEGUARD       11
#define DHD_STATUS_WRIST_INIT      12
#define DHD_STATUS_REDUNDANCY      13
#define DHD_STATUS_FORCEOFFCAUSE   14
#define DHD_STATUS_LOCKS           15

/* buttons count */
#define DHD_MAX_BUTTONS            16

/* velocity estimator computation mode */
#define DHD_VELOCITY_WINDOWING      0
#define DHD_VELOCITY_INSTANT        2
#define DHD_VELOCITY_WINDOW        20  // [ms]

/* USB operation modes */
#define DHD_COM_MODE_SYNC           0
#define DHD_COM_MODE_ASYNC          1
#define DHD_COM_MODE_VIRTUAL        3
#define DHD_COM_MODE_NETWORK        4

/* causes for device FORCE OFF state */
#define DHD_FORCEOFF_NONE           0
#define DHD_FORCEOFF_BUTTON         1
#define DHD_FORCEOFF_VELOCITY       2
#define DHD_FORCEOFF_WATCHDOG       3
#define DHD_FORCEOFF_SOFTWARE       4
#define DHD_FORCEOFF_USBDISCN       5
#define DHD_FORCEOFF_DEADMAN        6

/* thread management */
#define DHD_THREAD_PRIORITY_DEFAULT 0
#define DHD_THREAD_PRIORITY_HIGH    1
#define DHD_THREAD_PRIORITY_LOW     2


/****************************************************************************
 *  standard SDK
 ****************************************************************************/

  void        __SDK dhdEnableSimulator                   (bool on);
  int         __SDK dhdGetDeviceCount                    ();
  int         __SDK dhdGetAvailableCount                 ();
  int         __SDK dhdSetDevice                         (char ID);
  int         __SDK dhdGetDeviceID                       ();
  int         __SDK dhdGetSerialNumber                   (ushort *sn, char ID = -1);
  int         __SDK dhdOpen                              ();
  int         __SDK dhdOpenType                          (int type);
  int         __SDK dhdOpenSerial                        (int serial);
  int         __SDK dhdOpenID                            (char ID);
  int         __SDK dhdClose                             (char ID = -1);
  int         __SDK dhdStop                              (char ID = -1);
  int         __SDK dhdGetComMode                        (char ID = -1);                                                                                           /* added in release 3.3.2 */
  int         __SDK dhdEnableForce                       (uchar val, char ID = -1);
  int         __SDK dhdGetSystemType                     (char ID = -1);
  const char* __SDK dhdGetSystemName                     (char ID = -1);                                                                                           /* added in 3.2 release */
  int         __SDK dhdGetVersion                        (double *ver, char ID = -1);
  void        __SDK dhdGetSDKVersion                     (int *major, int *minor, int *release, int *revision);
  int         __SDK dhdGetStatus                         (int status[DHD_MAX_STATUS], char ID = -1);
  int         __SDK dhdGetDeviceAngleRad                 (double *angle, char ID = -1);
  int         __SDK dhdGetDeviceAngleDeg                 (double *angle, char ID = -1);
  int         __SDK dhdGetEffectorMass                   (double *mass,  char ID = -1);
  ulong       __SDK dhdGetSystemCounter                  ();
  int         __SDK dhdGetButton                         (int index, char ID = -1);
  uint        __SDK dhdGetButtonMask                     (char ID = -1);
  int         __SDK dhdSetOutput                         (uint output, char ID = -1);                                                                              /* added in 3.4 release */
  bool        __SDK dhdIsLeftHanded                      (char ID = -1);
  bool        __SDK dhdHasBase                           (char ID = -1);
  bool        __SDK dhdHasWrist                          (char ID = -1);
  bool        __SDK dhdHasActiveWrist                    (char ID = -1);
  bool        __SDK dhdHasGripper                        (char ID = -1);
  bool        __SDK dhdHasActiveGripper                  (char ID = -1);
  int         __SDK dhdReset                             (char ID = -1);
  int         __SDK dhdResetWrist                        (char ID = -1);
  int         __SDK dhdWaitForReset                      (int timeout = 0, char ID = -1);
  int         __SDK dhdSetStandardGravity                (double g, char ID = -1);
  int         __SDK dhdSetGravityCompensation            (int val = DHD_ON, char ID = -1);
  int         __SDK dhdSetBrakes                         (int val = DHD_ON, char ID = -1);
  int         __SDK dhdSetDeviceAngleRad                 (double angle, char ID = -1);
  int         __SDK dhdSetDeviceAngleDeg                 (double angle, char ID = -1);
  int         __SDK dhdSetEffectorMass                   (double mass,  char ID = -1);
  int         __SDK dhdGetPosition                       (double *px, double *py, double *pz, char ID = -1);
  int         __SDK dhdGetForce                          (double *fx, double *fy, double *fz, char ID = -1);
  int         __SDK dhdSetForce                          (double  fx, double  fy, double  fz, char ID = -1);
  int         __SDK dhdGetOrientationRad                 (double *oa, double *ob, double *og, char ID = -1);
  int         __SDK dhdGetOrientationDeg                 (double *oa, double *ob, double *og, char ID = -1);
  int         __SDK dhdGetPositionAndOrientationRad      (double *px, double *py, double *pz, double *oa, double *ob, double *og, char ID = -1);
  int         __SDK dhdGetPositionAndOrientationDeg      (double *px, double *py, double *pz, double *oa, double *ob, double *og, char ID = -1);
  int         __SDK dhdGetPositionAndOrientationFrame    (double *px, double *py, double *pz, double matrix[3][3], char ID = -1);
  int         __SDK dhdGetForceAndTorque                 (double *fx, double *fy, double *fz, double *tx, double *ty, double *tz, char ID = -1);
  int         __SDK dhdSetForceAndTorque                 (double  fx, double  fy, double  fz, double  tx, double  ty, double  tz, char ID = -1);
  int         __SDK dhdGetOrientationFrame               (double matrix[3][3], char ID = -1);
  int         __SDK dhdGetGripperAngleDeg                (double *a, char ID = -1);
  int         __SDK dhdGetGripperAngleRad                (double *a, char ID = -1);
  int         __SDK dhdGetGripperGap                     (double *g, char ID = -1);
  int         __SDK dhdGetGripperThumbPos                (double *px, double *py, double *pz,  char ID = -1);
  int         __SDK dhdGetGripperFingerPos               (double *px, double *py, double *pz,  char ID = -1);
  double      __SDK dhdGetComFreq                        (char ID = -1);
  int         __SDK dhdSetForceAndGripperForce           (double fx, double fy, double fz, double fg, char ID = -1);
  int         __SDK dhdSetForceAndTorqueAndGripperForce  (double fx, double fy, double fz, double tx, double ty, double tz, double fg, char ID = -1);              /* added in 3.3 release */
  int         __SDK dhdGetForceAndTorqueAndGripperForce  (double *fx, double *fy, double *fz, double *tx, double *ty, double *tz, double *f, char ID = -1);        /* added in 3.3 release */
  int         __SDK dhdConfigLinearVelocity              (int ms = DHD_VELOCITY_WINDOW, int mode = DHD_VELOCITY_WINDOWING, char ID = -1);
  int         __SDK dhdGetLinearVelocity                 (double *vx, double *vy, double *vz, char ID = -1);
  int         __SDK dhdConfigAngularVelocity             (int ms = DHD_VELOCITY_WINDOW, int mode = DHD_VELOCITY_WINDOWING, char ID = -1);
  int         __SDK dhdGetAngularVelocityRad             (double *wx, double *wy, double *wz, char ID = -1);
  int         __SDK dhdGetAngularVelocityDeg             (double *wx, double *wy, double *wz, char ID = -1);
  int         __SDK dhdConfigGripperVelocity             (int ms = DHD_VELOCITY_WINDOW, int mode = DHD_VELOCITY_WINDOWING, char ID = -1);
  int         __SDK dhdGetGripperLinearVelocity          (double *vg, char ID = -1);
  int         __SDK dhdGetGripperAngularVelocityRad      (double *wg, char ID = -1);
  int         __SDK dhdGetGripperAngularVelocityDeg      (double *wg, char ID = -1);
  int         __SDK dhdEmulateButton                     (uchar val, char ID = -1);
  int         __SDK dhdGetBaseAngleXRad                  (double *angle, char ID = -1);                                                                            /* added in release 3.3 */
  int         __SDK dhdGetBaseAngleXDeg                  (double *angle, char ID = -1);                                                                            /* added in release 3.3 */
  int         __SDK dhdSetBaseAngleXRad                  (double  angle, char ID = -1);                                                                            /* added in release 3.3.1 */
  int         __SDK dhdSetBaseAngleXDeg                  (double  angle, char ID = -1);                                                                            /* added in release 3.3.1 */
  int         __SDK dhdGetBaseAngleZRad                  (double *angle, char ID = -1);                                                                            /* added in release 3.3 */
  int         __SDK dhdGetBaseAngleZDeg                  (double *angle, char ID = -1);                                                                            /* added in release 3.3 */
  int         __SDK dhdSetBaseAngleZRad                  (double  angle, char ID = -1);                                                                            /* added in release 3.3 */
  int         __SDK dhdSetBaseAngleZDeg                  (double  angle, char ID = -1);                                                                            /* added in release 3.3 */
  int         __SDK dhdSetVibration                      (double freq, double amplitude, int type = 0, char ID = -1);                                              /* added in release 3.7 */


/****************************************************************************
 *  expert SDK
 ****************************************************************************/

  int         __SDK dhdEnableExpertMode                  ();
  int         __SDK dhdDisableExpertMode                 ();
  int         __SDK dhdPreset                            (int val[DHD_MAX_DOF], uchar mask, char ID = -1);
  int         __SDK dhdCalibrateWrist                    (char ID = -1);
  int         __SDK dhdSetTimeGuard                      (int us,  char ID = -1);
  int         __SDK dhdSetVelocityThreshold              (uint val, char ID = -1);
  int         __SDK dhdGetVelocityThreshold              (uint *val, char ID = -1);
  int         __SDK dhdUpdateEncoders                    (char ID = -1);
  int         __SDK dhdGetDeltaEncoders                  (int *enc0, int *enc1, int *enc2, char ID = -1);
  int         __SDK dhdGetWristEncoders                  (int *enc0, int *enc1, int *enc2, char ID = -1);
  int         __SDK dhdGetGripperEncoder                 (int *enc, char ID = -1);
  int         __SDK dhdGetEncoder                        (int index, char ID = -1);
  int         __SDK dhdSetMotor                          (int index, ushort val, char ID = -1);
  int         __SDK dhdSetDeltaMotor                     (ushort mot0, ushort mot1, ushort mot2, char ID = -1);
  int         __SDK dhdSetWristMotor                     (ushort mot0, ushort mot1, ushort mot2, char ID = -1);
  int         __SDK dhdSetGripperMotor                   (ushort mot, char ID = -1);
  int         __SDK dhdDeltaEncoderToPosition            (int  enc0, int  enc1, int  enc2, double *px, double *py, double *pz, char ID = -1);
  int         __SDK dhdDeltaPositionToEncoder            (double px, double py, double pz, int  *enc0, int  *enc1, int  *enc2, char ID = -1);
  int         __SDK dhdDeltaMotorToForce                 (ushort mot0, ushort mot1, ushort mot2, int enc0, int enc1, int enc2, double  *fx, double  *fy, double  *fz, char ID = -1);
  int         __SDK dhdDeltaForceToMotor                 (double  fx, double  fy, double  fz, int enc0, int enc1, int enc2, ushort *mot0, ushort *mot1, ushort *mot2, char ID = -1);
  int         __SDK dhdWristEncoderToOrientation         (int  enc0, int  enc1, int  enc2, double *oa, double *ob, double *og, char ID = -1);
  int         __SDK dhdWristOrientationToEncoder         (double oa, double ob, double og, int  *enc0, int  *enc1, int  *enc2, char ID = -1);
  int         __SDK dhdWristMotorToTorque                (ushort mot0, ushort mot1, ushort mot2, int enc0, int enc1, int enc2, double  *tx, double  *ty, double  *tz, char ID = -1);
  int         __SDK dhdWristTorqueToMotor                (double  ta, double  tb, double  tg, int enc0, int enc1, int enc2, ushort *mot0, ushort *mot1, ushort *mot2, char ID = -1);
  int         __SDK dhdGripperEncoderToAngleRad          (int enc, double *a, char ID = -1);
  int         __SDK dhdGripperEncoderToGap               (int enc, double *g, char ID = -1);
  int         __SDK dhdGripperAngleRadToEncoder          (double a, int *enc, char ID = -1);
  int         __SDK dhdGripperGapToEncoder               (double g, int *enc, char ID = -1);
  int         __SDK dhdGripperMotorToForce               (ushort mot, double *f, int e[4], char ID = -1);
  int         __SDK dhdGripperForceToMotor               (double f, ushort *mot, int e[4], char ID = -1);
  int         __SDK dhdSetMot                            (ushort mot[DHD_MAX_DOF], uchar mask = 0xff, char ID = -1);
  int         __SDK dhdPreloadMot                        (ushort mot[DHD_MAX_DOF], uchar mask = 0xff, char ID = -1);                                               /* added in release 3.7 */
  int         __SDK dhdGetEnc                            (int    enc[DHD_MAX_DOF], uchar mask = 0xff, char ID = -1);
  int         __SDK dhdSetBrk                            (uchar mask = 0xff, char ID = -1);
  int         __SDK dhdGetDeltaJointAngles               (double *j0, double *j1, double *j2, char ID = -1);                                                       /* added in release 3.3   */
  int         __SDK dhdGetDeltaJacobian                  (double jcb[3][3], char ID = -1);                                                                         /* added in release 3.3   */
  int         __SDK dhdDeltaJointAnglesToJacobian        (double j0, double j1, double j2, double jcb[3][3], char ID = -1);                                        /* added in release 3.3   */
  int         __SDK dhdDeltaJointTorquesExtrema          (double j0, double j1, double j2, double minq[3], double maxq[3], char ID = -1);                          /* added in release 3.3   */
  int         __SDK dhdDeltaGravityJointTorques          (double j0, double j1, double j2, double *q0, double *q1, double *q2, char ID = -1);                      /* added in release 3.3   */
  int         __SDK dhdSetDeltaJointTorques              (double t0, double t1, double t2, char ID = -1);                                                          /* added in release 3.3   */
  int         __SDK dhdGetJointAngles                    (double j[DHD_MAX_DOF], char ID = -1);                                                                    /* added in release 3.3   */
  int         __SDK dhdJointAnglesToInertiaMatrix        (double j[DHD_MAX_DOF], double inertia[6][6], char ID = -1);                                              /* added in release 3.3   */
  int         __SDK dhdSetComMode                        (int mode, char ID = -1);                                                                                 /* added in release 3.3.2 */
  int         __SDK dhdSetComModePriority                (int priority, char ID = -1);                                                                             /* added in release 3.3.2 */
  int         __SDK dhdSetWatchdog                       (unsigned char  val, char ID = -1);                                                                       /* added in release 3.3.2 */
  int         __SDK dhdGetWatchdog                       (unsigned char *val, char ID = -1);                                                                       /* added in release 3.3.2 */


/****************************************************************************
 *  controller SDK
 ****************************************************************************/

  int         __SDK dhdControllerSetDevice               (int device, char ID = -1);
  int         __SDK dhdReadConfigFromFile                (char *filename, char ID = -1);


/****************************************************************************
 *  OS independent utilities
 ****************************************************************************/

  bool        __SDK dhdKbHit                             ();
  char        __SDK dhdKbGet                             ();
  double      __SDK dhdGetTime                           ();
  void        __SDK dhdSleep                             (double sec);
  int         __SDK dhdStartThread                       (void *func(void *), void *arg, int priority);


#ifdef __cplusplus
}
#endif


#endif
