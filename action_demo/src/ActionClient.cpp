#include<action_demo/DoDishesAction.h>
#include<actionlib/client/simple_action_client.h>
#include<ros/ros.h>
typedef actionlib::SimpleActionClient<action_demo::DoDishesAction> Client;

class DoDishesActionClient
{
public:
    DoDishesActionClient(const std::string client_name,bool flag=true):client(client_name,flag)
    {

    }

    void Start()
    {
        client.waitForServer();
        action_demo::DoDishesGoal goal;
        goal.dishwasher_id=1;

        client.sendGoal(goal,boost::bind(&DoDishesActionClient::DoneCb,this,_1,_2),
                             boost::bind(&DoDishesActionClient::ActiveCb,this),
                             boost::bind(&DoDishesActionClient::FeedbackCb,this,_1)
                    );
        
        client.waitForResult(ros::Duration(10.0));

        if(client.getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
            printf("目标发送成功，正在执行\n");
        else
        {
            printf("发送失败。。。\n");
            client.cancelAllGoals();
        }
        std::cout<<"当前的状态是"<<client.getState().toString()<<std::endl;
    }
private:
    void DoneCb(const actionlib::SimpleClientGoalState& state,const action_demo::DoDishesResultConstPtr& result)
    {
        printf("已经完成，现在的状态是%s\n",state.toString().c_str());
        printf("总共已经洗完了%d个：\n",result->total_dishes_cleaded);
        ros::shutdown();
    }
    void ActiveCb()
    {
        printf("此处是激活了\n");
    }
    void FeedbackCb(const action_demo::DoDishesFeedbackConstPtr& feedback)
    {
        printf("反馈函数的结果，目前的工作百分比为%f\n",feedback->percent_complete);
    }


private:
    Client client;
};

int main(int argc,char** argv)
{
    ros::init(argc, argv, "do_dishes_client");
    DoDishesActionClient actionClient("do_dishes",true);
    actionClient.Start();
    ros::spin();


    return 0;
}