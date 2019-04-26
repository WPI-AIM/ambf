#ifndef AMBFDEFINES_H
#define AMBFDEFINES_H

//---------------------------------------------------------------------------
#include "chai3d.h"
#include "ambf.h"
//---------------------------------------------------------------------------
#include <GLFW/glfw3.h>
#include <boost/program_options.hpp>
#include <mutex>
#include <thread>
#include <map>
#include <termios.h>
//---------------------------------------------------------------------------
using namespace ambf;
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------
#include "CBullet.h"

enum AMBFCmdType {_jp, _jw, _cp, _cw, _null};
enum AMBFCmdMode {freefall, homing, dancing};

struct AMBFCmd
{
	bool updated;		// raven command updated
	AMBFCmdType type;	// raven command type

	vector<float> js;  	// raven joint space command (position or wrench: depending on AMBFCmdType)
	tf::Vector3   cf;	// raven cartesian force command
	tf::Vector3   ct;	// raven cartesian torque command
	bool cn_flag;		// raven state: children name flag
	bool jn_flag;		// raven state: joint name flag
	bool jp_flag;		// raven state: joint position flag
};

struct AMBFSta
{
	bool updated;			 // raven state updated

	vector<float> jp;        // raven joint space position state
    tf::Transform cp;        // raven catesian space position state
    tf::Vector3   cf;        // raven cartesian force state     
    tf::Vector3   ct;        // raven cartesian torque state      
};


class AMBFDef {
public:

  static const int 			   raven_joints;
  static const int 			   raven_arms;
  static const int 			   loop_rate;
  static const string          sub_append;          // place holder for namescpace strings
  static const string          pub_append;  
  static const string          raven_append;
  static const vector<string>  arm_append;          // left arm 0 & right arm 1

  static const tf::Vector3             zero_vec;    // place holder for frequently used arrays
  static const vector<float>           zero_joints;
  static const vector<float>           max_joints;
  static const vector<float>           min_joints;
  static const vector<float>           home_joints;
  static const vector<unsigned char>   true_joints;
  static const vector<unsigned char>   false_joints;

};

#endif