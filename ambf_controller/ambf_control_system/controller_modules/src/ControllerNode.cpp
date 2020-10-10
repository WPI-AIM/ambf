
#include "controller_modules/ControllerNode.h"



ControllerNode::ControllerNode(rigidBodyPtr _handle,
                               ros::NodeHandle* _nh,
                               const Eigen::Ref<const Eigen::MatrixXd>& kp,
                               const Eigen::Ref<const Eigen::MatrixXd>& kd): handle(_handle), n(*_nh), controller(kp, kd)
{

    client_ID = n.serviceClient<rbdl_server::RBDLInverseDynamics>("InverseDynamics");
    have_path = false;

}

void ControllerNode::setGain(const Eigen::Ref<const Eigen::MatrixXd>& Kp, const Eigen::Ref<const Eigen::MatrixXd>& Kd)
{
    controller.setKd(Kd);
    controller.setKp(Kp);
}

void ControllerNode::updataPath(const trajectory_generator::trajectory& new_path)
{
        path = new_path;
        path_index = 0;
        path_length = new_path.pathLength;

}

bool ControllerNode::startController()
{
    boost::thread thread_foo(boost::bind(&ControllerNode::control, this));
}

///
/// \brief RBDLServer::VectToEigen
/// \param msg
///
Eigen::VectorXd ControllerNode::VectToEigen(const std::vector<double> &msg)
{
    std::vector<double> vec(msg.begin(), msg.end());
    Eigen::VectorXd Q =  Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec.data(), vec.size());
    return Q;
}

void ControllerNode::control()
{

    std::vector<double> tau;
    Eigen::VectorXd pos; //=  VectToEigen(pos_vec);
    Eigen::VectorXd vel; // =  VectToEigen(vel_vec);
    Eigen::VectorXd e, ed, aq;
    rbdl_server::RBDLInverseDynamics Invdny_msg;
    std::vector<double> curr_pos, curr_vel;
    ros::Rate loop_rate(1000);

    while(ros::ok())
    {
        std::vector<float> pos_vec = handle->get_all_joint_pos();
        std::vector<float> vel_vec = handle->get_all_joint_vel();

        //check if there is a new path
        if(have_path)
        {

            if(path_index == path_length)
            {
                have_path = false;
            }
            else
            {
                desired_pos = VectToEigen( path.traj[path_index].position );
                desired_vel = VectToEigen( path.traj[path_index].velocity );
                path_index++;
            }

        }

        // calculate the control input
        e = desired_pos - VectToEigen(curr_pos);
        ed = desired_vel - VectToEigen(curr_vel);
        controller.calculate(e, ed, aq);
        std::vector<double> aq_vect(&aq[0], aq.data()+aq.cols()*aq.rows());

        //build the message
        Invdny_msg.request.q = curr_pos;
        Invdny_msg.request.qd = curr_vel;
        Invdny_msg.request.qdd = aq_vect;

        //call the service
        if (client_ID.call(Invdny_msg))
        {
            //get torque
            //sent torqe to AMBF
           //  tau = Invdny_msg.Response.tau;
            std::vector<float> float_tau(tau.begin(), tau.end());
            handle->set_joint_efforts(float_tau);
        }
        else
        {
            ROS_ERROR("Failed to call service ID");

        }

        loop_rate.sleep();
        ros::spinOnce();
    }



}


