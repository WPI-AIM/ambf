#ifndef AMBFDEFINES_H
#define AMBFDEFINES_H

//---------------------------------------------------------------------------
#include "chai3d.h"
#include "ambf.h"
//---------------------------------------------------------------------------
#include <GLFW/glfw3.h>
#include <boost/program_options.hpp>
#include <mutex>
#include <map>
//---------------------------------------------------------------------------
using namespace ambf;
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------
#include "CBullet.h"

enum AMBFCmdType {_jp, _jw, _cp, _cw, _null};

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

#endif