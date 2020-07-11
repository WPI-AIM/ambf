#include "ambf_client.h"
#include<ros/ros.h>
#include <ros/master.h>

int main(int argc, char* argv[])
{
    Client client;
    client.connect();
	return 0;
}
