
#include "controller_modules/ControllerNode.h"



ControllerNode::ControllerNode(rigidBodyPtr _handle,
                               ros::NodeHandle* _nh,
                               PDController& controller): handle(_handle), n(*_nh), controller(controller)
{

    running = false;
    start_controller = n.subscribe("start_controller", 1000, &ControllerNode::startControllerCallback, this);
    stop_controller = n.subscribe("stop_controller", 1000, &ControllerNode::stopControllerCallback, this);
    client_ID = n.serviceClient<rbdl_server::RBDLInverseDynamics>("InverseDynamics");
    desired_pub = n.advertise<sensor_msgs::JointState>("desiredJoint", 1000);
    have_path = false;
    step_count = 0;
    mtx_.unlock();

}

Eigen::VectorXd ControllerNode::getDesiredPos(){return desired_pos;}
Eigen::VectorXd ControllerNode::getDesiredVel(){return desired_vel;}
Eigen::VectorXd ControllerNode::getDesiredAccel(){return desired_accel;}
int ControllerNode::getStepCount(){return step_count;}
int ControllerNode::getPathLength(){return path_length;}
bool ControllerNode::isRunning(){return running;}

void ControllerNode::startControllerCallback(const std_msgs::Empty )
{
   if(running)
   {
       ROS_INFO("Controller is already running");
   }
   else
   {
       startController();
   }
}

void ControllerNode::stopControllerCallback(const std_msgs::Empty )
{
    if(running)
    {
        ROS_INFO("Stoping the controller");
        running = false;
    }
    else
    {
        ROS_INFO("Controller is not running");
    }
}

void ControllerNode::updataPath(const trajectory_generator::trajectory& new_path)
{
    boost::lock_guard<boost::mutex> lock{mtx_};
    path = new_path;
    step_count = 0;
    path_length = new_path.traj.size();
    have_path = true;

}

bool ControllerNode::startController()
{

    if (!running)
    {
        ROS_INFO("Starting the Controller");
        run = boost::thread(boost::bind(&ControllerNode::controlloop, this));
        running = true;
    }

    return true;

}

///
/// \brief RBDLServer::VectToEigen
/// \param msg
///
template<typename T, typename A>
Eigen::VectorXd ControllerNode::VectToEigen(std::vector<T,A> const& msg )
{
    std::vector<double> vec(msg.begin(), msg.end());
    Eigen::VectorXd Q =  Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec.data(), vec.size());
    return Q;
}


void ControllerNode::step()
{
    if(step_count == path_length - 1)
    {
        have_path = false;
    }
    else
    {
        for(int i =0; i <path.traj[step_count].position.size(); i++ )
        {
            //std::cout<<path.traj[step_count].position ;
        }
        desired_pos = VectToEigen( path.traj[step_count].position );
        desired_vel = VectToEigen( path.traj[step_count].velocity );
        step_count++;
    }
}


std::vector<double> ControllerNode::calcTorque(const std::vector<double> pos, const std::vector<double> vel)
{

    // calculate the control input
       boost::lock_guard<boost::mutex> lock{mtx_};
    std::cout<<"desired pos"<<desired_pos.rows()<<std::endl;
    std::cout<<"desired vel"<<desired_vel.rows()<<std::endl;
    std::cout<<"pos "<<VectToEigen(pos).size()<<std::endl;
    std::cout<<"vel "<<VectToEigen(vel).size()<<std::endl;
    desired_pos - VectToEigen(pos);
    std::cout<<"hello\nz";
    Eigen::VectorXd e = desired_pos - VectToEigen(pos);

    std::cout<<"hello2\n";
    Eigen::VectorXd ed = desired_vel - VectToEigen(vel);
    std::cout<<"ch1"<<std::endl;

    Eigen::VectorXd aq;
    controller.calc_tau(e, ed, aq);
    std::vector<double> aq_vect(&aq[0], aq.data()+aq.cols()*aq.rows());
    return aq_vect;

}

void ControllerNode::updateState()
{
    std::vector<float> pos_vec = handle->get_all_joint_pos();
    std::vector<float> vel_vec = handle->get_all_joint_vel();

    curr_pos = std::vector<double>(pos_vec.begin(), pos_vec.end());
    curr_vel = std::vector<double>(vel_vec.begin(), vel_vec.end());
}

void ControllerNode::controlloop()
{

    sensor_msgs::JointState desired_msg;
    Eigen::VectorXd pos; //=  VectToEigen(pos_vec);
    Eigen::VectorXd vel; // =  VectToEigen(vel_vec);
    std::vector<double> aq;
    rbdl_server::RBDLInverseDynamics Invdny_msg;
    ros::Rate loop_rate(1000);
    //set then inital desired state to the current state
    std::vector<float> pos_vec = handle->get_all_joint_pos();
    std::vector<float> vel_vec = handle->get_all_joint_vel();
    std::vector<double> pos_vec_temp(pos_vec.begin(), pos_vec.end());
    std::vector<double> vel_vec_temp(vel_vec.begin(), vel_vec.end());

    desired_pos =  VectToEigen(pos_vec_temp);
    desired_vel =  VectToEigen(vel_vec_temp);

    //run the controller in a loop
    while(ros::ok() && running)
    {

        updateState();
        //check if there is a new path
        boost::lock_guard<boost::mutex> lock{mtx_};
        if(have_path)
        {
            step();
        }
//        else
//        {
//            std::vector<float> pos_vec = handle->get_all_joint_pos();
//            std::vector<float> vel_vec = handle->get_all_joint_vel();
//            std::vector<double> pos_vec_temp(pos_vec.begin(), pos_vec.end());
//            std::vector<double> vel_vec_temp(vel_vec.begin(), vel_vec.end());

//            desired_pos =  VectToEigen(pos_vec_temp);
//            desired_vel =  VectToEigen(vel_vec_temp);
//        }

        aq = calcTorque(curr_pos, curr_vel);

        //build the message
        Invdny_msg.request.q = curr_pos;
        Invdny_msg.request.qd = curr_vel;
        Invdny_msg.request.qdd = aq;

        std::vector<double> pos_msg(&desired_pos[0], desired_pos.data()+desired_pos.cols()*desired_pos.rows());
        std::vector<double> vel_msg(&desired_vel[0], desired_vel.data()+desired_vel.cols()*desired_vel.rows());
        desired_msg.position = pos_msg;
        desired_msg.velocity = vel_msg;
        desired_msg.effort = aq;

       //call the service
        if (client_ID.call(Invdny_msg))
        {
            //get torque
            //sent torqe to AMBF
//            std::vector<double> tau = Invdny_msg.response.tau;
//            for(int i=0; i < tau.size(); i++)
//                std::cout << tau.at(i) << ' ';
//            std::vector<float> float_tau(tau.begin(), tau.end());
//            desired_pub.publish(desired_msg);
            //handle->set_joint_efforts(float_tau);

        }
        else
        {
            ROS_ERROR("Failed to call service ID");

        }

        loop_rate.sleep();
        ros::spinOnce();
    }


}


