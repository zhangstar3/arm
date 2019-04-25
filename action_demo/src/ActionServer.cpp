#include<action_demo/DoDishesAction.h>
#include<actionlib/server/simple_action_server.h>
#include<ros/ros.h>
typedef actionlib::SimpleActionServer<action_demo::DoDishesAction> Server;

class DoDishesActionServer
{
public:
    DoDishesActionServer(ros::NodeHandle n):server(n,"do_dishes",boost::bind(&DoDishesActionServer::ExecuteCb,this,_1),false)
    {
        server.registerPreemptCallback(boost::bind(&DoDishesActionServer::preemptCb,this));
    }

    void Start()
    {
        server.start();
    }

    void ExecuteCb(const action_demo::DoDishesGoalConstPtr& goal)
    {
        printf("获取到了目标值，它的ID是：%d\n",goal->dishwasher_id);
        action_demo::DoDishesFeedback feedback; //反馈对象
        ros::Rate rate(1);
        int cur_finished_i=1;
        int toal_dish_num=10;

        for(cur_finished_i=1;cur_finished_i<=toal_dish_num;cur_finished_i++)
        {
            if(server.isActive()==false)
                break;
            printf("已经洗完了%d个：\n",cur_finished_i);
            feedback.percent_complete=(float)cur_finished_i/toal_dish_num;
            server.publishFeedback(feedback);
            rate.sleep();
        }

        action_demo::DoDishesResult result;
        result.total_dishes_cleaded=cur_finished_i;
        if(server.isActive())
            server.setSucceeded();
    }

    void preemptCb()
    {
        if(server.isActive())
            server.setPreempted();
    }

private:
    Server server;
};

int main(int argc,char** argv)
{
    ros::init(argc,argv,"do_dishes_server");
    ros::NodeHandle n;
    DoDishesActionServer actionServer(n);
    actionServer.Start();
    ros::spin();


    return 0;
}