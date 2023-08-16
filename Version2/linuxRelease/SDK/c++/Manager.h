/*
将ang_dif_range 设置为0；
*/

#ifndef __MANAGER_H
#define __MANAGER_H

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include<stdio.h> 
#include<algorithm>

using namespace std;

Logger managerLog(".\\", "text", 1, 10, 0);

/******************** 各种声明 ********************/
enum actions{ // 机器人的行动
    ACT_FORWARD,    // 前进（米/秒）[-2, 6]
    ACT_ROTATE,     // 旋转速度（弧度/秒）[-π,π]，正值表示逆时针旋转
    ACT_BUY,        // 购买当前工作台的物品
    ACT_SELL,       // 出售当前物品给当前工作台
    ACT_DESTROY     // 摧毁物品
};  

struct Robot{   // 机器人属性
    uint8_t ID; // 机器人序号
    char worktableID=-1; // 所处工作台ID，-1表示没有
    char objectID=0;  // 携带物品ID，0表示没有，[1,7]为对应物品
    float val_time=0; // 时间价值系数[0.8, 1]，0表示无物品
    float val_crash=0;    // 碰撞价值系数[0.8, 1]，0表示无物品
    float v_rad=0;  // 角速度（弧度/秒），正数表示顺时针
    struct {  // 线速度（米/秒）
        double x;
        double y;
    }v_line;
    float direct=0;   // 朝向[-π,π]；0，右方向；π/2，上方向
    struct{     // 坐标
        double x;
        double y;
    }loc;

    char order = 0; //任务 0没任务 1买任务 2卖任务
    vector<pair<double,uint8_t> > distances;    //机器人与工作台距离 
    bool arrived = false;
    int target_table=-1;


    
};

struct Worktable{   // 工作台属性
    uint8_t ID; // 序号
    uint8_t classID;    // 类别[1,9]
    int last=-1;   // 剩余生产时间(帧数)；-1，没有生产；0，生产因输出格满而阻塞；>=0，剩余生产帧数
    uint16_t sta_material=0; // 原材料格状态，二进制表示；如 48(000110000) 表示拥有物品4和5。
    uint8_t sta_produce=0; // 产品格状态；0，无；1，有
    struct{     // 坐标
        double x;
        double y;
    } loc;
    vector<int> robot={-1,-1,-1,-1};//表示未与任何机器人配对
    int robot_link=4;
};

class Manager{
    public:
        /**
         * @brief 解析地图信息以初始化
         * 
         * @param mapStr 接收到的地图信息
         */
        Manager(vector<string> mapStr);
        
        /**
         * @brief 解析帧信息
         * 
         * @param frameStr 接受到的帧信息
         */
        void praseFrameStr(vector<string> frameStr);

        /**
         * @brief 机器人行为控制
         * 
         * @param robotID 机器人ID
         * @param action 行为，详见 @enum actions
         * @param intensity 对应的操作值，如果不需要就不使用该变量即可
         * @ps: 该函数调用完不会执行，要调用doActions()函数进行enable
         */
        void control(uint8_t robotID, actions action, float intensity=0);

        /**
         * @brief 发送控制指令
         * 
         */
        void doActions();
        /**
         * @brief 机器人与工作台距离
         * 
         * @param m_robot 机器人
         * @param m_worktable 工作台
         */
        double getDistance(Robot m_robot, Worktable m_worktable);

        /**
         * @brief 机器人与机器人距离
         * 
         * @param m_robot1 机器人1
         * @param m_robot2 机器人2
         */
        double getDistR2R(Robot m_robot1, Robot m_robot2);

        /**
         * @brief 机器人与工作台距离
         * 
         */
        void robotTableDistance();

        /**
         * @brief 机器人避障碍
         * @param robotID 机器人ID
         * @return 机器人旋转角度
         */
        float avoid_obs(int8_t robotID);


        /**
         * @brief 依次由最大的工作台到最小的工作台与四个机器人进行排序
         * 
         */
        vector<pair<uint8_t, Worktable>> distanceSort();
        bool set_Pointv4(uint8_t robotID,double ref_x,double ref_y);
        void robot_to_table();
        int find_recenttable(int robot_id);
        void robot_action();
        void statis_enough();
        void statis_table();
        void robot_exchange();
        Robot m_robots[4];  // 四台机器人
        vector<Worktable> m_worktables; // 工作台
        vector<int> table_norobot={-1,-1,-1,-1};

        int m_frameID; //帧序号
        uint32_t m_money;   // 当前金钱数
        uint8_t m_amount_workTable; // 工作台数量

        int class_1_enough=0;
        int class_2_enough=0;
        int class_3_enough=0;
        int class_4_enough=0;
        int class_5_enough=0;
        int class_6_enough=0;
        int class_7_enough=0;

        int no_7=0;

    protected:
        vector<string> m_ctlStr;  // 控制指令
};

/**
 * @brief 读取字符串直到OK
 * 
 * @param strings 收到的数据
 * @return true 成功
 * @return false 失败
 */
bool readUntilOK(vector<string> &strings);



/******************** 函数定义 ********************/
Manager::Manager(vector<string> mapStr){
    string obj;
    int cnt_wTable=0, cnt_robot=0;

    // 扫描地图上的物体
    for(int l=0; l<mapStr.size(); l++){
        obj = mapStr[l];
        for(int x=0; x<obj.size(); x++){
            char pt = obj[x];
            if(pt == '.'){  // 空
                continue;
            }else if(pt == 'A'){  // 机器人
                m_robots[cnt_robot].ID = cnt_robot;
                m_robots[cnt_robot].loc.x = float(x)*0.5 + 0.25;
                m_robots[cnt_robot].loc.y = float(99-l)*0.5 + 0.25;
                cnt_robot++;
            }else if((pt >= '1') && (pt <= '9')){   // 工作台
                Worktable wTable;
                wTable.ID = cnt_wTable;
                wTable.classID = pt - '0';
                wTable.loc.x = float(x)*0.5 + 0.25;
                wTable.loc.y = float(99-l)*0.5 + 0.25;
                m_worktables.push_back(wTable);
                cnt_wTable++;

                m_amount_workTable = cnt_wTable;
            }
        }
    }
}


void Manager::praseFrameStr(vector<string> frameStr){
    istringstream one_line;	// 输入流
    string obj;

    // 1行-帧序号，金钱数
    one_line.str(frameStr[0]);
    one_line >> obj;
    m_frameID = stoul(obj);
    one_line >> obj;
    m_money = stoul(obj);

    // 2行-工作台数量(跳过)
    // 3~3+K行-工作台信息
    for(int i=0; i<m_amount_workTable; i++){
        one_line.str(frameStr[i+2]);
        one_line >> obj;
        // m_worktables[i].classID = obj[0] - '0';
        one_line >> obj;
        // m_worktables[i].loc.x = stod(obj);
        one_line >> obj;
        // m_worktables[i].loc.y = stod(obj);
        one_line >> obj;
        m_worktables[i].last = stoi(obj);
        one_line >> obj;
        m_worktables[i].sta_material = stoi(obj);
        one_line >> obj;
        m_worktables[i].sta_produce = stoul(obj);
    }

    // 剩余4行-机器人信息
    for(int i=0; i<4; i++){
        one_line.str(frameStr[i+2+m_amount_workTable]);
        one_line >> obj;
        m_robots[i].worktableID = stoi(obj);
        one_line >> obj;
        m_robots[i].objectID = stoi(obj);
        one_line >> obj;
        m_robots[i].val_time = stof(obj);
        one_line >> obj;
        m_robots[i].val_crash = stof(obj);
        one_line >> obj;
        m_robots[i].v_rad = stod(obj);
        one_line >> obj;
        m_robots[i].v_line.x = stod(obj);
        one_line >> obj;
        m_robots[i].v_line.y = stod(obj);
        one_line >> obj;
        m_robots[i].direct = stod(obj);
        one_line >> obj;
        m_robots[i].loc.x = stod(obj);
        one_line >> obj;
        m_robots[i].loc.y = stod(obj);
    }
}

void Manager::control(uint8_t robotID, actions action, float intensity){
    char str[100];
    switch(action){
        case ACT_FORWARD:
            sprintf(str, "forward %d %f\n", robotID, intensity);
            break;
        case ACT_ROTATE:
            sprintf(str, "rotate %d %f\n", robotID, intensity);        
            break;
        case ACT_BUY:
            sprintf(str, "buy %d\n", robotID);  
            break;
        case ACT_SELL:
            sprintf(str, "sell %d\n", robotID);  
            break;
        case ACT_DESTROY:
            sprintf(str, "destroy %d\n", robotID);  
            break;
        default:
            return;
    }
    m_ctlStr.push_back(string(str));    // 输入到控制指令列表中
}

void Manager::doActions(){
    printf("%d\n", m_frameID);    // 发送帧ID

    while(!m_ctlStr.empty()){
        cout << m_ctlStr.back();
        m_ctlStr.pop_back();
    }

    printf("OK\n"); // 报文尾
    fflush(stdout);
}

/**********获取距离**********/
double Manager::getDistance(Robot m_robot, Worktable m_worktable)
{
    double distance = 0;
    double robot_x, robot_y;
    double worktable_x, worktable_y;

    robot_x = m_robot.loc.x;
    robot_y = m_robot.loc.y;

    worktable_x = m_worktable.loc.x;
    worktable_y = m_worktable.loc.y;

    distance = sqrt((worktable_x-robot_x)*(worktable_x-robot_x) + (worktable_y-robot_y)*(worktable_y-robot_y));
    return distance;
}

/**********获取机器人之间的距离**********/
double Manager::getDistR2R(Robot m_robot1, Robot m_robot2)
{
    double distance = 0;
    double robot1_x, robot1_y;
    double robot2_x, robot2_y;

    robot1_x = m_robot1.loc.x;
    robot1_y = m_robot1.loc.y;

    robot2_x = m_robot2.loc.x;
    robot2_y = m_robot2.loc.y;

    distance = sqrt((robot2_x-robot1_x)*(robot2_x-robot1_x) + (robot2_y-robot1_y)*(robot2_y-robot1_y));
    return distance;
}

/**********获取所有机器人与所有工作台距离**********/
void Manager::robotTableDistance()
{
    //managerLog.addLog(to_string(m_frameID));
    
    for(int i = 0; i < 4; i++)
    {   
        m_robots[i].distances.clear();
        for(int j = 0; j < m_worktables.size(); j++)
        {
           double dis = getDistance(m_robots[i], m_worktables[j]); //获取机器人与每个工作台的距离，存入容器distances中
           uint8_t tableID = m_worktables[j].classID;
           
           m_robots[i].distances.push_back(make_pair(dis, tableID));
           //managerLog.addLog(to_string(i) + ", " + to_string(j) + ", " + to_string(m_robots[i].distances[j].first));
           /*if(m_robots[i].distances[j].first < 0.4)
           {
               m_robots[i].worktableID = j;
           }*/
        }
    }
}


/**********依次由最大的工作台到最小的工作台与四个机器人进行排序**********/
vector<pair<uint8_t, Worktable>> Manager::distanceSort()
{
    managerLog.addLog(to_string(m_frameID));
    vector<pair<uint8_t, Worktable>> vp_sort;

    robotTableDistance();
    for(int i = 0; i < m_worktables.size(); i++)
    {
        uint8_t robotId = 0;
        double dist= 100;
        for(int j = 0; j < 4; j++)
        {
            if(m_robots[j].distances[i].first < dist)
            {
                robotId = j;
                dist = m_robots[j].distances[i].first;
                //vp_sort.push_back(make_pair(robotId, m_worktables[i]));
                
            }
        }
        vp_sort.push_back(make_pair(robotId, m_worktables[i]));
        managerLog.addLog(to_string(robotId) + ", " +  to_string(m_worktables[i].ID));
    }


    for (int  i = 0; i < m_worktables.size(); i++)
    {
        for(int j = 0; j < m_worktables.size()-i-1;j++)
        {
            
            if(vp_sort[j].second.classID < vp_sort[j+1].second.classID)
            {
                pair<uint8_t, Worktable> temp;
                temp = vp_sort[j];
                vp_sort[j] = vp_sort[j+1];
                vp_sort[j+1] = temp;
            }
        }
    }
    managerLog.addLog("GOOD");
    for(int  i = 0; i < m_worktables.size(); i++)
    {
        managerLog.addLog(to_string(vp_sort[i].first) + " , " + to_string(vp_sort[i].second.classID));
    }
    
    return vp_sort;
}

void Manager::statis_table()
{
    int num7=0;
    for(int i=0;i<m_amount_workTable;i++)
    {
        if(m_worktables[i].classID==7) num7++;
    }
    if(num7==0) no_7=1;
}
void Manager::statis_enough()
{
    int table_class1=0;
    int table_class2=0;
    int table_class3=0;
    int table_class4=0;
    int table_class5=0;
    int table_class6=0;
    int table_class7=1;
    for(int i=0;i<m_amount_workTable;i++)
    {
        if(m_worktables[i].classID==4)
        {
            if((m_worktables[i].sta_material & (1<<1))==0) table_class1++;
            if((m_worktables[i].sta_material & (1<<2))==0) table_class2++;
        }
        if(m_worktables[i].classID==5)
        {
            if((m_worktables[i].sta_material & (1<<1))==0) table_class1++;
            if((m_worktables[i].sta_material & (1<<3))==0) table_class3++;
        }
        if(m_worktables[i].classID==6)
        {
            if((m_worktables[i].sta_material & (1<<2))==0) table_class2++;
            if((m_worktables[i].sta_material & (1<<3))==0) table_class3++;
        }
        if(no_7)
        {
            table_class4=100;
            table_class5=100;
            table_class6=100;
        }
        else
        {
            if(m_worktables[i].classID==7)
            {
                if((m_worktables[i].sta_material & (1<<4))==0) table_class4++;
                if((m_worktables[i].sta_material & (1<<5))==0) table_class5++;
                if((m_worktables[i].sta_material & (1<<6))==0) table_class6++;
            }


        }

    }
    for(int i=0;i<4;i++)
    {
    //    if(m_robots[i].objectID==0)
    //    {
    //        if(m_worktables[m_robots[i].target_table].classID==1)//即将去买1
    //        {
    //            table_class1--;
    //        }
    //        if(m_worktables[m_robots[i].target_table].classID==2)//即将去买2
    //        {
    //            table_class2--;
    //        }
    //        if(m_worktables[m_robots[i].target_table].classID==3)//即将去买3
    //        {
    //            table_class3--;
    //        }
    //        if(m_worktables[m_robots[i].target_table].classID==4)//即将去买4
    //        {
    //            table_class4--;
    //        }
    //        if(m_worktables[m_robots[i].target_table].classID==5)//即将去买5
    //        {
    //            table_class5--;
    //        }
    //        if(m_worktables[m_robots[i].target_table].classID==6)//即将去买6
    //        {
    //            table_class6--;
    //        }
    //    }
        if(m_robots[i].objectID!=0)
        {
            if(m_robots[i].objectID==1) table_class1--;
            if(m_robots[i].objectID==2) table_class2--;
            if(m_robots[i].objectID==3) table_class3--;
            if(m_robots[i].objectID==4) table_class4--;
            if(m_robots[i].objectID==5) table_class5--;
            if(m_robots[i].objectID==6) table_class6--;
        }
       
    }
    if(table_class1<=0) class_1_enough=1;
    else class_1_enough=0;
    if(table_class2<=0) class_2_enough=1;
    else class_2_enough=0;
    if(table_class3<=0) class_3_enough=1;
    else class_3_enough=0;
    if(table_class4<=0) class_4_enough=1;
    else class_4_enough=0;
    if(table_class5<=0) class_5_enough=1;
    else class_5_enough=0;
    if(table_class6<=0) class_6_enough=1;
    else class_6_enough=0;
    if(table_class7<=0) class_7_enough=1;
    else class_7_enough=0;

    //managerLog.addLog("flag: "+ to_string(class_1_enough) + ", " +  to_string(class_2_enough)+" , " +to_string(class_3_enough));

}
int Manager::find_recenttable(int robot_id)
{
    int target_id=-1;

    if(m_robots[robot_id].objectID==0)//如果没有带，此刻空闲可以取
    {
        float dis_normal=10000, dis_temp=10000, dis_seven = 10000,dis_six = 10000, dis_five = 10000, dis_four = 10000;
        int target_seven = -1,target_six = -1, target_five = -1,target_four = -1, target_normal = -1;
        int extra_dis = 100;          //重要参数      当7类工作台检测到只缺其中一种原材料时，是否检测越过该距离搜索缺少材料，当为0时，不搜索。当为100时，全图搜索
        vector<pair<float, int>> distAndCnt;

        int last_time = 0;          //重要参数      提前预测7类工作台的产出成果的时间。当为0时，表示不预测
        int last_time2 = 0;         //重要参数      提前预测6.5.4类工作台的参数成果时间。当为0时，表示不预测
        //如果7类工作台有成果
        for(int i=0;i<m_amount_workTable;i++) {
            if (m_worktables[i].classID == 7 && (m_worktables[i].sta_produce||(m_worktables[i].last < last_time && m_worktables[i].last != -1))  && m_worktables[i].robot_link >= robot_id) {
                
                dis_temp = sqrt((m_robots[robot_id].loc.x - m_worktables[i].loc.x) *
                                (m_robots[robot_id].loc.x - m_worktables[i].loc.x)
                                + (m_robots[robot_id].loc.y - m_worktables[i].loc.y) *
                                  (m_robots[robot_id].loc.y - m_worktables[i].loc.y));
                if (dis_temp < dis_seven) {
                    dis_seven = dis_temp;
                    target_seven = i;
                }
            }
        }
        if(dis_seven != 10000)
        {
            distAndCnt.push_back(make_pair(dis_seven, target_seven));
        }

        //如果6类工作台有成果
        for(int i=0;i<m_amount_workTable;i++) {
            if ((m_worktables[i].classID == 6) && (m_worktables[i].sta_produce||(m_worktables[i].last < last_time2 && m_worktables[i].last != -1)) && m_worktables[i].robot_link >= robot_id) {
                if (class_1_enough && m_worktables[i].classID == 1) continue;
                if (class_2_enough && m_worktables[i].classID == 2) continue;
                if (class_3_enough && m_worktables[i].classID == 3) continue;
                if (class_4_enough && m_worktables[i].classID == 4) continue;
                if (class_5_enough && m_worktables[i].classID == 5) continue;
                if (class_6_enough && m_worktables[i].classID == 6) continue;
                if (class_7_enough && m_worktables[i].classID == 7) continue;
                dis_temp = sqrt((m_robots[robot_id].loc.x - m_worktables[i].loc.x) *
                                (m_robots[robot_id].loc.x - m_worktables[i].loc.x)
                                + (m_robots[robot_id].loc.y - m_worktables[i].loc.y) *
                                  (m_robots[robot_id].loc.y - m_worktables[i].loc.y));
                if (dis_temp < dis_six) {
                    dis_six = dis_temp;
                    target_six = i;
                }
            }
        }
        if(dis_six != 10000)
        {
            distAndCnt.push_back(make_pair(dis_six, target_six));
        }

        //如果5类工作台有成果
        for(int i=0;i<m_amount_workTable;i++) {
            if ((m_worktables[i].classID == 5) && (m_worktables[i].sta_produce||(m_worktables[i].last < last_time2 && m_worktables[i].last != -1)) && m_worktables[i].robot_link >= robot_id) {
                if (class_1_enough && m_worktables[i].classID == 1) continue;
                if (class_2_enough && m_worktables[i].classID == 2) continue;
                if (class_3_enough && m_worktables[i].classID == 3) continue;
                if (class_4_enough && m_worktables[i].classID == 4) continue;
                if (class_5_enough && m_worktables[i].classID == 5) continue;
                if (class_6_enough && m_worktables[i].classID == 6) continue;
                if (class_7_enough && m_worktables[i].classID == 7) continue;
                dis_temp = sqrt((m_robots[robot_id].loc.x - m_worktables[i].loc.x) *
                                (m_robots[robot_id].loc.x - m_worktables[i].loc.x)
                                + (m_robots[robot_id].loc.y - m_worktables[i].loc.y) *
                                  (m_robots[robot_id].loc.y - m_worktables[i].loc.y));
                if (dis_temp < dis_five) {
                    dis_five = dis_temp;
                    target_five = i;
                }
            }
        }
        if(dis_five != 10000)
        {
            distAndCnt.push_back(make_pair(dis_five, target_five));
        }

        //如果4类工作台有成果
        for(int i=0;i<m_amount_workTable;i++) {
            if ((m_worktables[i].classID == 4) && (m_worktables[i].sta_produce||(m_worktables[i].last < last_time2 && m_worktables[i].last != -1)) && m_worktables[i].robot_link >= robot_id) {
                if (class_1_enough && m_worktables[i].classID == 1) continue;
                if (class_2_enough && m_worktables[i].classID == 2) continue;
                if (class_3_enough && m_worktables[i].classID == 3) continue;
                if (class_4_enough && m_worktables[i].classID == 4) continue;
                if (class_5_enough && m_worktables[i].classID == 5) continue;
                if (class_6_enough && m_worktables[i].classID == 6) continue;
                if (class_7_enough && m_worktables[i].classID == 7) continue;
                dis_temp = sqrt((m_robots[robot_id].loc.x - m_worktables[i].loc.x) *
                                (m_robots[robot_id].loc.x - m_worktables[i].loc.x)
                                + (m_robots[robot_id].loc.y - m_worktables[i].loc.y) *
                                  (m_robots[robot_id].loc.y - m_worktables[i].loc.y));
                if (dis_temp < dis_four) {
                    dis_four = dis_temp;
                    target_four = i;
                }
            }
        }
        if(dis_four != 10000)
        {
            distAndCnt.push_back(make_pair(dis_four, target_four));
        }

        //如果3、2、1类工作台有成果
        for(int i=0;i<m_amount_workTable;i++){
            if((m_worktables[i].classID==1 || m_worktables[i].classID==2 || m_worktables[i].classID==3) && m_worktables[i].sta_produce )//如果有货   && m_worktables[i].robot_link>=robot_id       //提醒，解决机器人不能同时去同一个3.2.1类工作台取货物的问题
            {
                if(class_1_enough && m_worktables[i].classID==1) continue;
                if(class_2_enough && m_worktables[i].classID==2) continue;
                if(class_3_enough && m_worktables[i].classID==3) continue;
                if(class_4_enough && m_worktables[i].classID==4) continue;
                if(class_5_enough && m_worktables[i].classID==5) continue;
                if(class_6_enough && m_worktables[i].classID==6) continue;
                dis_temp=sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                        +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y));
                if(dis_temp<dis_normal)
                {
                    dis_normal=dis_temp;
                    target_normal=i;
                }
            }
        }
        if(dis_normal != 10000)
        {
            distAndCnt.push_back(make_pair(dis_normal, target_normal));
        }

        for(int i=0; i<distAndCnt.size(); i++)
        {
            for(int j = 0;j < distAndCnt.size()-i-1; j++)
            {
                if(distAndCnt[j].first > distAndCnt[j+1].first)
                {
                    pair<float, int> temp;
                    temp = distAndCnt[j];
                    distAndCnt[j] = distAndCnt[j+1];
                    distAndCnt[j+1] = temp;
                }
            }
        }

        //搜索7类工作台是否只缺一种原材料
        for(int i=0;i<m_amount_workTable;i++)
        {
            if(m_worktables[i].classID == 7 &&
                ((m_worktables[i].sta_material & (1<<4)) && (m_worktables[i].sta_material & (1<<5)) && ((m_worktables[i].sta_material & (1<<6)) == 0)) )
            {
                if(distAndCnt.size())
                {
                    if(dis_six < distAndCnt[0].first + extra_dis)
                    {
                        target_id = target_six;
                        managerLog.addLog(" six is good ");
                        return target_id;
                    }
                }
                
            }
            
            else if(m_worktables[i].classID == 7 &&
                ((m_worktables[i].sta_material & (1<<4)) && ((m_worktables[i].sta_material & (1<<5)) == 0) && (m_worktables[i].sta_material & (1<<6))) )
            {
                if(distAndCnt.size())
                {
                    if(dis_five < distAndCnt[0].first + extra_dis)
                    {
                        target_id = target_five;
                        managerLog.addLog(" five is good ");
                        return target_id;
                    }
                }
                
            }

            else if(m_worktables[i].classID == 7 &&
                (((m_worktables[i].sta_material & (1<<4)) == 0) && (m_worktables[i].sta_material & (1<<5)) && (m_worktables[i].sta_material & (1<<6))) )
            {
                if(distAndCnt.size())
                {
                    if(dis_four < distAndCnt[0].first + extra_dis)
                    {
                        target_id = target_four;
                        managerLog.addLog(" four is good ");
                        return target_id;
                    }
                }
                
            }
            else
            {
                if(distAndCnt.size())
                    target_id = distAndCnt[0].second;
            }   
        }

        
        //最后10s的情况
        if(m_frameID >= 8000)
        {
            for(int i=0;i<m_amount_workTable;i++) 
            {
                if (m_worktables[i].classID == 7 && (m_worktables[i].sta_produce||(m_worktables[i].last < last_time && m_worktables[i].last != -1)) && m_worktables[i].robot_link>=robot_id)        //提醒：相比于temp32，将 <= 改为 <
                {
                    return i;
                }
            }
        }
        

        if(target_id!=-1)//找到最近的有货的
        {
            return target_id;
        }
    }
    if(m_robots[robot_id].objectID!=0  && m_robots[robot_id].target_table==-1)//如果带了，找最近的可以卖的
    {
        //int last_time3 = 0;         //重要参数      提前预测7.6.5.4类工作台提前出产结果的时间，并有该时间判断，该产品所缺少的原材料。当为0时，极为不预测。
        vector<pair<float,int>> dis;
        vector<pair<float,int>> dis_hasone;
        for(int i=0;i<m_amount_workTable;i++)
        {
            if(m_worktables[i].classID>3)
            {
                if(m_robots[robot_id].objectID==3)
                {
                    if(m_worktables[i].classID==6 || m_worktables[i].classID==5)
                    {
                        /*if( (((m_worktables[i].last <last_time3 && m_worktables[i].last != -1) && (!(m_worktables[i].sta_produce && (m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID)))) && (!(((m_worktables[i].sta_material & (1<<1))==0) && ((m_worktables[i].sta_material & (1<<2))==0) && (m_worktables[i].sta_material & (1<<3)))) ) || ((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0))
                        && (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=3))//如果缺货*/
                        if(((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0) && ((m_worktables[i].sta_material & (1<<1))==0 && (m_worktables[i].sta_material & (1<<2))==0) && (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=3) )//如果缺货
                        {
                            dis.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                                     +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y)), i));
                        }
                        if( (m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0 && ((m_worktables[i].sta_material & (1<<1))||(m_worktables[i].sta_material & (1<<2))) && (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=3))
                        {
                            dis_hasone.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                                     +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y)), i));
                        }
                    }
                }
                if(m_robots[robot_id].objectID==1)
                {
                    if(m_worktables[i].classID==4 || m_worktables[i].classID==5)
                    {
                        /*if( (((m_worktables[i].last <last_time3 && m_worktables[i].last != -1) && (!(m_worktables[i].sta_produce && (m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID)))) && (!(((m_worktables[i].sta_material & (1<<2))==0) && ((m_worktables[i].sta_material & (1<<3))==0) && (m_worktables[i].sta_material & (1<<1)))) ) || ((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0))
                        && (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=1))//如果缺货*/
                        if(((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0) && ((m_worktables[i].sta_material & (1<<2))==0&&(m_worktables[i].sta_material & (1<<3))==0) && (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=1))//如果缺货
                        {
                            dis.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                                     +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y)), i));
                        }
                        if( (m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0 && ((m_worktables[i].sta_material & (1<<2))||(m_worktables[i].sta_material & (1<<3))) && (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=1))
                        {
                            dis_hasone.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                                     +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y)), i));
                        }
                        
                    }
                }

                if(m_robots[robot_id].objectID==2)
                {
                    if(m_worktables[i].classID==4 || m_worktables[i].classID==6)
                    {
                        /*if( (((m_worktables[i].last <last_time3 && m_worktables[i].last != -1) && (!(m_worktables[i].sta_produce && (m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID)))) && (!(((m_worktables[i].sta_material & (1<<1))==0) && ((m_worktables[i].sta_material & (1<<3))==0) && (m_worktables[i].sta_material & (1<<2)))) ) || ((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0))
                        && (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=2))//如果缺货*/
                        if(((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0)&& ((m_worktables[i].sta_material & (1<<1))==0&&(m_worktables[i].sta_material & (1<<3))==0) && (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=2))//如果缺货
                        {
                            dis.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                                     +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y)), i));
                        }
                        if( (m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0 && ((m_worktables[i].sta_material & (1<<1))||(m_worktables[i].sta_material & (1<<3))) && (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=2))
                        {
                            dis_hasone.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                                     +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y)), i));
                        }

                    }
                }

                if(m_robots[robot_id].objectID==4)
                {
                    if(no_7)
                    {
                        if(m_worktables[i].classID==9)
                        {
                            if(((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0))//如果缺货
                            {
                                dis.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                                     +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y)), i));

                            }
                        }
                    }
                    else
                    {
                        if(m_worktables[i].classID==7)
                        {
                            /*if( (((m_worktables[i].last <last_time3 && m_worktables[i].last != -1) && (!(m_worktables[i].sta_produce && (m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID)))) && (!(((m_worktables[i].sta_material & (1<<5))==0) && ((m_worktables[i].sta_material & (1<<6))==0) && (m_worktables[i].sta_material & (1<<4)))) && (!((m_worktables[i].sta_material & (1<<5)) && ((m_worktables[i].sta_material & (1<<6))==0) && (m_worktables[i].sta_material & (1<<4)) )) && (!(((m_worktables[i].sta_material & (1<<5))==0) && (m_worktables[i].sta_material & (1<<6)) && (m_worktables[i].sta_material & (1<<4)) )) )
                             || ((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0))
                             && (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=4))//如果缺货*/
                            if(((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0)&& ((m_worktables[i].sta_material & (1<<5))==0||(m_worktables[i].sta_material & (1<<6))==0) &&(m_worktables[i].robot_link==4 || m_robots[m_worktables[i].robot_link].objectID!=4))//如果缺货
                            {
                                dis.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                                     +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y)), i));
                            }
                            if( (m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0 && ((m_worktables[i].sta_material & (1<<5))&&(m_worktables[i].sta_material & (1<<6))) && (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=4))
                            {
                                dis_hasone.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                                        +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y)), i));
                            }
                        }
                    }

                }
                if(m_robots[robot_id].objectID==5)
                {
                    if(no_7)
                    {
                        if(m_worktables[i].classID==9)
                        {
                            if(((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0))//如果缺货
                            {
                                dis.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                                     +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y)), i));

                            }
                        }
                    }
                    else
                    {
                        if(m_worktables[i].classID==7)
                        {
                            /*if( (((m_worktables[i].last <last_time3 && m_worktables[i].last != -1) && (!(m_worktables[i].sta_produce && (m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID)))) && (!(((m_worktables[i].sta_material & (1<<4))==0) && ((m_worktables[i].sta_material & (1<<6))==0) && (m_worktables[i].sta_material & (1<<5)))) && (!((m_worktables[i].sta_material & (1<<4)) && ((m_worktables[i].sta_material & (1<<6))==0) && (m_worktables[i].sta_material & (1<<5)) )) && (!(((m_worktables[i].sta_material & (1<<4))==0) && (m_worktables[i].sta_material & (1<<6)) && (m_worktables[i].sta_material & (1<<5)) )) )
                             || ((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0))
                             && (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=5))//如果缺货*/
                            if(((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0)&& ((m_worktables[i].sta_material & (1<<4))==0||(m_worktables[i].sta_material & (1<<6))==0) && (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=5))//如果缺货
                            {
                                dis.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                                     +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y)), i));
                            }
                            if( (m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0 && ((m_worktables[i].sta_material & (1<<4))&&(m_worktables[i].sta_material & (1<<6))) && (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=5))
                            {
                                dis_hasone.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                                        +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y)), i));
                            }
                        }
                    }

                }
                if(m_robots[robot_id].objectID==6)
                {
                    if(no_7)
                    {
                        if(m_worktables[i].classID==9)
                        {
                            if(((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0))//如果缺货
                            {
                                dis.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                                     +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y)), i));
                            }
                        }
                    }
                    else
                    {
                        if(m_worktables[i].classID==7)
                        {
                            /*if( (((m_worktables[i].last <last_time3 && m_worktables[i].last != -1) && (!(m_worktables[i].sta_produce && (m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID)))) && (!(((m_worktables[i].sta_material & (1<<4))==0) && ((m_worktables[i].sta_material & (1<<5))==0) && (m_worktables[i].sta_material & (1<<6)))) && (!((m_worktables[i].sta_material & (1<<4)) && ((m_worktables[i].sta_material & (1<<5))==0) && (m_worktables[i].sta_material & (1<<6)) )) && (!(((m_worktables[i].sta_material & (1<<4))==0) && (m_worktables[i].sta_material & (1<<5)) && (m_worktables[i].sta_material & (1<<6)) )) )
                             || ((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0))
                             && (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=6))//如果缺货*/
                            if(((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0)&& ((m_worktables[i].sta_material & (1<<5))==0||(m_worktables[i].sta_material & (1<<4))==0) && (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=6))//如果缺货
                            {
                                dis.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                                     +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y)), i));
                            }
                            if( (m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0 && ((m_worktables[i].sta_material & (1<<5))&&(m_worktables[i].sta_material & (1<<4))) && (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=6))
                            {
                                dis_hasone.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                                        +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y)), i));
                            }
                        }
                    }

                }
                if(m_robots[robot_id].objectID==7)
                {
                    if(m_worktables[i].classID==8)
                    {
                        if(((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0))//如果缺货                   //提醒，修改了连续拿7 ，解决第二个7需要等第一个7送完才开始送的问题
                        {
                            dis.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                                     +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y)), i));
                        }
                    }
                }



            }
        }
        int target_temp0=-1;
        int target_temp1=-1;

        if(dis.size()>=1)
        {
            for(int i=0; i<dis.size(); i++)
            {
                for(int j = 0;j < dis.size()-i-1; j++)
                {
                    if(dis[j].first > dis[j+1].first)
                    {
                        pair<float, int> temp;
                        temp = dis[j];
                        dis[j] = dis[j+1];
                        dis[j+1] = temp;
                    }
                }
            }

            for(int i=0; i<dis_hasone.size(); i++)
            {
                for(int j = 0;j < dis_hasone.size()-i-1; j++)
                {
                    if(dis_hasone[j].first > dis_hasone[j+1].first)
                    {
                        pair<float, int> temp;
                        temp = dis_hasone[j];
                        dis_hasone[j] = dis_hasone[j+1];
                        dis_hasone[j+1] = temp;
                    }
                }
            }
            //sort(dis.begin(),dis.end());
            /*if(dis.size()==1)
            {
                target_id = dis[0].second;
            }*/
            if(dis.size() > 0 && dis_hasone.size() ==0)
            {
                target_id = dis[0].second;
            }
            else if(dis_hasone.size() > 0 && dis.size() ==0)
            {
                target_id = dis_hasone[0].second;
            }
            else if(dis_hasone.size() >0 && dis.size() > 0)
            {
                    target_temp0 = dis[0].second;
                    target_temp1 = dis_hasone[0].second;
                    
                    if(abs(target_temp0-target_temp1)>=100)
                    {
                        target_id=target_temp0;
                    }
                    else
                    {
                        if(m_robots[robot_id].objectID==1)
                        {
                             if(((m_worktables[target_temp1].sta_material & (1<<2))||(m_worktables[target_temp1].sta_material & (1<<3)))&& ((m_worktables[target_temp0].sta_material & (1<<2))==0&&(m_worktables[target_temp0].sta_material & (1<<3))==0))
                            {
                                target_id=target_temp1;
                            }
                             else
                             {
                                 target_id=target_temp0;
                             }
                        }
                        if(m_robots[robot_id].objectID==2)
                        {
                            if(((m_worktables[target_temp1].sta_material & (1<<1))||(m_worktables[target_temp1].sta_material & (1<<3)))&& ((m_worktables[target_temp0].sta_material & (1<<1))==0&&(m_worktables[target_temp0].sta_material & (1<<3))==0))
                            {
                                target_id=target_temp1;
                            }
                            else
                            {
                                target_id=target_temp0;
                            }
                        }
                        if(m_robots[robot_id].objectID==3)
                        {
                            if(((m_worktables[target_temp1].sta_material & (1<<1))||(m_worktables[target_temp1].sta_material & (1<<2)))&& ((m_worktables[target_temp0].sta_material & (1<<1))==0&&(m_worktables[target_temp0].sta_material & (1<<2))==0))
                            {
                                target_id=target_temp1;
                            }
                            else
                            {
                                target_id=target_temp0;
                            }
                        }
                        if(m_robots[robot_id].objectID==4)
                        {
                            if(((m_worktables[target_temp1].sta_material & (1<<5))&&(m_worktables[target_temp1].sta_material & (1<<6)))&& ((m_worktables[target_temp0].sta_material & (1<<5))==0||(m_worktables[target_temp0].sta_material & (1<<6))==0))
                            {
                                target_id=target_temp1;
                            }
                            else
                            {
                                target_id=target_temp0;
                            }
                        }
                        if(m_robots[robot_id].objectID==5)
                        {
                            if(((m_worktables[target_temp1].sta_material & (1<<4))&&(m_worktables[target_temp1].sta_material & (1<<6)))&& ((m_worktables[target_temp0].sta_material & (1<<4))==0||(m_worktables[target_temp0].sta_material & (1<<6))==0))
                            {
                                target_id=target_temp1;
                            }
                            else
                            {
                                target_id=target_temp0;
                            }
                        }
                        if(m_robots[robot_id].objectID==6)
                        {
                            if(((m_worktables[target_temp1].sta_material & (1<<5))&&(m_worktables[target_temp1].sta_material & (1<<4)))&& ((m_worktables[target_temp0].sta_material & (1<<5))==0||(m_worktables[target_temp0].sta_material & (1<<4))==0))
                            {
                                target_id=target_temp1;
                            }
                            else
                            {
                                target_id=target_temp0;
                            }
                        }
                        if(m_robots[robot_id].objectID==7)
                        {

                                target_id=target_temp0;
                        }
                    }
            }
        }
        if(target_id!=-1)//找到最近的有货的
        {
            return target_id;
        }
    }
    return -1;
}
void Manager::robot_action()
{
    for(int i=0;i<4;i++)
    {
        int tar_id=-1;
        if(m_robots[i].objectID==0)
        {
            tar_id= find_recenttable(i);
            if(tar_id!=-1)
            {
                m_worktables[tar_id].robot_link=i;
                m_robots[i].target_table=tar_id;
                m_robots[i].order=1;//去买
            }
        }
        if(m_robots[i].objectID!=0)
        {
            tar_id= find_recenttable(i);
            if(tar_id!=-1)
            {
                m_worktables[tar_id].robot_link=i;
                m_robots[i].target_table=tar_id;
                m_robots[i].order=2;//去卖
            }

        }
    }

}
void Manager::robot_to_table()
{
    managerLog.addLog(to_string(m_frameID));
   for(int i=0;i<4;i++)
   {
       if(m_robots[i].target_table!=-1)//有目标工作台
       {
           set_Pointv4(i,m_worktables[m_robots[i].target_table].loc.x,m_worktables[m_robots[i].target_table].loc.y);
            managerLog.addLog("robot: " + to_string(i) + " class: " + to_string(m_worktables[m_robots[i].target_table].classID));
           if(m_robots[i].arrived)
           {
               if(m_robots[i].order==1)
               {
                   if(m_frameID <= 8850)
                   {
                       control(m_robots[i].ID, ACT_BUY);
                   }
               }
               if(m_robots[i].order==2)
               {
                   control(m_robots[i].ID, ACT_SELL);
               }
               m_robots[i].order=0;
               m_worktables[m_robots[i].target_table].robot_link=4;
               m_robots[i].target_table=-1;

           }
       }

   }

}

void Manager::robot_exchange()
{
    vector<pair<double, int>> distance;
    vector<pair<double, int>> distance1;
    vector<pair<double, int>> distance2;
    vector<pair<double, int>> distance3;
    vector<pair<double, int>> distance4;
    vector<pair<double, int>> distance5;
    vector<pair<double, int>> distance6;
    vector<pair<double, int>> distance7;

    for(int cnt_rob = 0; cnt_rob < 4; cnt_rob++)
    {
        if(m_robots[cnt_rob].objectID == 0)
        {
            int tar_id = m_robots[cnt_rob].target_table;
            double dist = getDistance(m_robots[cnt_rob], m_worktables[tar_id]);
            distance.push_back(make_pair(dist, cnt_rob));
        }
        if(m_robots[cnt_rob].objectID == 1)
        {
            int tar_id = m_robots[cnt_rob].target_table;
            double dist = getDistance(m_robots[cnt_rob], m_worktables[tar_id]);
            distance1.push_back(make_pair(dist, cnt_rob));
        }
        if(m_robots[cnt_rob].objectID == 2)
        {
            int tar_id = m_robots[cnt_rob].target_table;
            double dist = getDistance(m_robots[cnt_rob], m_worktables[tar_id]);
            distance2.push_back(make_pair(dist, cnt_rob));
        }
        if(m_robots[cnt_rob].objectID == 3)
        {
            int tar_id = m_robots[cnt_rob].target_table;
            double dist = getDistance(m_robots[cnt_rob], m_worktables[tar_id]);
            distance3.push_back(make_pair(dist, cnt_rob));
        }
        if(m_robots[cnt_rob].objectID == 4)
        {
            int tar_id = m_robots[cnt_rob].target_table;
            double dist = getDistance(m_robots[cnt_rob], m_worktables[tar_id]);
            distance4.push_back(make_pair(dist, cnt_rob));
        }
        if(m_robots[cnt_rob].objectID == 5)
        {
            int tar_id = m_robots[cnt_rob].target_table;
            double dist = getDistance(m_robots[cnt_rob], m_worktables[tar_id]);
            distance5.push_back(make_pair(dist, cnt_rob));
        }
        if(m_robots[cnt_rob].objectID == 6)
        {
            int tar_id = m_robots[cnt_rob].target_table;
            double dist = getDistance(m_robots[cnt_rob], m_worktables[tar_id]);
            distance6.push_back(make_pair(dist, cnt_rob));
        }
        if(m_robots[cnt_rob].objectID == 7)
        {
            int tar_id = m_robots[cnt_rob].target_table;
            double dist = getDistance(m_robots[cnt_rob], m_worktables[tar_id]);
            distance7.push_back(make_pair(dist, cnt_rob));
        }
    }
    if(distance.size() >= 2)
    {
        managerLog.addLog("exchange distance0");
        double now_dis = distance[0].first + distance[1].first;
        double dist1 = getDistance(m_robots[distance[0].second], m_worktables[m_robots[distance[1].second].target_table]);
        double dist2 = getDistance(m_robots[distance[1].second], m_worktables[m_robots[distance[0].second].target_table]);
        double fur_dis = dist1 + dist2;
        managerLog.addLog("cnt_rob == " + to_string(distance[0].second) + " now_dis == " + to_string(now_dis));
        managerLog.addLog("cnt_rob == " + to_string(distance[1].second) + " fur_dis == " + to_string(fur_dis));
        if(now_dis > fur_dis)
        {
            int temp_table = m_robots[distance[0].second].target_table;
            int temp_robot = m_worktables[m_robots[distance[0].second].target_table].robot_link;

            m_robots[distance[0].second].target_table = m_robots[distance[1].second].target_table;
            m_worktables[m_robots[distance[0].second].target_table].robot_link = m_worktables[m_robots[distance[1].second].target_table].robot_link;

            m_robots[distance[1].second].target_table = temp_table;
            m_worktables[m_robots[distance[1].second].target_table].robot_link = temp_robot;
            managerLog.addLog("exchange distance0 happened");
            
        }
    }
    if(distance1.size() >= 2)
    {
        managerLog.addLog("exchange distance1");
        double now_dis = distance1[0].first + distance1[1].first;
        double dist1 = getDistance(m_robots[distance1[0].second], m_worktables[m_robots[distance1[1].second].target_table]);
        double dist2 = getDistance(m_robots[distance1[1].second], m_worktables[m_robots[distance1[0].second].target_table]);
        double fur_dis = dist1 + dist2;
        managerLog.addLog("cnt_rob == " + to_string(distance1[0].second) + " now_dis == " + to_string(now_dis));
        managerLog.addLog("cnt_rob == " + to_string(distance1[1].second) + " fur_dis == " + to_string(fur_dis));
        if(now_dis > fur_dis)
        {
            int temp_table = m_robots[distance1[0].second].target_table;
            int temp_robot = m_worktables[m_robots[distance1[0].second].target_table].robot_link;

            m_robots[distance1[0].second].target_table = m_robots[distance1[1].second].target_table;
            m_worktables[m_robots[distance1[0].second].target_table].robot_link = m_worktables[m_robots[distance1[1].second].target_table].robot_link;

            m_robots[distance1[1].second].target_table = temp_table;
            m_worktables[m_robots[distance1[1].second].target_table].robot_link = temp_robot;
            managerLog.addLog("exchange distance1 happened");
        }
    }
    if(distance2.size() >= 2)
    {
        managerLog.addLog("exchange distance2");
        double now_dis = distance2[0].first + distance2[1].first;
        double dist1 = getDistance(m_robots[distance2[0].second], m_worktables[m_robots[distance2[1].second].target_table]);
        double dist2 = getDistance(m_robots[distance2[1].second], m_worktables[m_robots[distance2[0].second].target_table]);
        double fur_dis = dist1 + dist2;
        managerLog.addLog("cnt_rob == " + to_string(distance2[0].second) + " now_dis == " + to_string(now_dis));
        managerLog.addLog("cnt_rob == " + to_string(distance2[1].second) + " fur_dis == " + to_string(fur_dis));
        if(now_dis > fur_dis)
        {
            int temp_table = m_robots[distance2[0].second].target_table;
            int temp_robot = m_worktables[m_robots[distance2[0].second].target_table].robot_link;

            m_robots[distance2[0].second].target_table = m_robots[distance2[1].second].target_table;
            m_worktables[m_robots[distance2[0].second].target_table].robot_link = m_worktables[m_robots[distance2[1].second].target_table].robot_link;

            m_robots[distance2[1].second].target_table = temp_table;
            m_worktables[m_robots[distance2[1].second].target_table].robot_link = temp_robot;
            managerLog.addLog("exchange distance2 happened");
        }
    }
    if(distance3.size() >= 2)
    {
        managerLog.addLog("exchange distance3");
        double now_dis = distance3[0].first + distance3[1].first;
        double dist1 = getDistance(m_robots[distance3[0].second], m_worktables[m_robots[distance3[1].second].target_table]);
        double dist2 = getDistance(m_robots[distance3[1].second], m_worktables[m_robots[distance3[0].second].target_table]);
        double fur_dis = dist1 + dist2;
        managerLog.addLog("cnt_rob == " + to_string(distance3[0].second) + " now_dis == " + to_string(now_dis));
        managerLog.addLog("cnt_rob == " + to_string(distance3[1].second) + " fur_dis == " + to_string(fur_dis));
        if(now_dis > fur_dis)
        {
            int temp_table = m_robots[distance3[0].second].target_table;
            int temp_robot = m_worktables[m_robots[distance3[0].second].target_table].robot_link;

            m_robots[distance3[0].second].target_table = m_robots[distance3[1].second].target_table;
            m_worktables[m_robots[distance3[0].second].target_table].robot_link = m_worktables[m_robots[distance3[1].second].target_table].robot_link;

            m_robots[distance3[1].second].target_table = temp_table;
            m_worktables[m_robots[distance3[1].second].target_table].robot_link = temp_robot;
            managerLog.addLog("exchange distance3 happened");
        }
    }
    if(distance4.size() >= 2)
    {
        managerLog.addLog("exchange distance4");
        double now_dis = distance4[0].first + distance4[1].first;
        double dist1 = getDistance(m_robots[distance4[0].second], m_worktables[m_robots[distance4[1].second].target_table]);
        double dist2 = getDistance(m_robots[distance4[1].second], m_worktables[m_robots[distance4[0].second].target_table]);
        double fur_dis = dist1 + dist2;
        if(now_dis > fur_dis)
        {
            int temp_table = m_robots[distance4[0].second].target_table;
            int temp_robot = m_worktables[m_robots[distance4[0].second].target_table].robot_link;

            m_robots[distance4[0].second].target_table = m_robots[distance4[1].second].target_table;
            m_worktables[m_robots[distance4[0].second].target_table].robot_link = m_worktables[m_robots[distance4[1].second].target_table].robot_link;

            m_robots[distance4[1].second].target_table = temp_table;
            m_worktables[m_robots[distance4[1].second].target_table].robot_link = temp_robot;
            managerLog.addLog("exchange distance4 happened");
        }
    }
    if(distance5.size() >= 2)
    {
        double now_dis = distance5[0].first + distance5[1].first;
        double dist1 = getDistance(m_robots[distance5[0].second], m_worktables[m_robots[distance5[1].second].target_table]);
        double dist2 = getDistance(m_robots[distance5[1].second], m_worktables[m_robots[distance5[0].second].target_table]);
        double fur_dis = dist1 + dist2;
        if(now_dis > fur_dis)
        {
            managerLog.addLog("exchange distance5");
            int temp_table = m_robots[distance5[0].second].target_table;
            int temp_robot = m_worktables[m_robots[distance5[0].second].target_table].robot_link;

            m_robots[distance5[0].second].target_table = m_robots[distance5[1].second].target_table;
            m_worktables[m_robots[distance5[0].second].target_table].robot_link = m_worktables[m_robots[distance5[1].second].target_table].robot_link;

            m_robots[distance5[1].second].target_table = temp_table;
            m_worktables[m_robots[distance5[1].second].target_table].robot_link = temp_robot;
            managerLog.addLog("exchange distance5 happened");
        }
    }
    if(distance6.size() >= 2)
    {
        double now_dis = distance6[0].first + distance6[1].first;
        double dist1 = getDistance(m_robots[distance6[0].second], m_worktables[m_robots[distance6[1].second].target_table]);
        double dist2 = getDistance(m_robots[distance6[1].second], m_worktables[m_robots[distance6[0].second].target_table]);
        double fur_dis = dist1 + dist2;
        if(now_dis > fur_dis)
        {
            managerLog.addLog("exchange distance6");
            int temp_table = m_robots[distance6[0].second].target_table;
            int temp_robot = m_worktables[m_robots[distance6[0].second].target_table].robot_link;

            m_robots[distance6[0].second].target_table = m_robots[distance6[1].second].target_table;
            m_worktables[m_robots[distance6[0].second].target_table].robot_link = m_worktables[m_robots[distance6[1].second].target_table].robot_link;

            m_robots[distance6[1].second].target_table = temp_table;
            m_worktables[m_robots[distance6[1].second].target_table].robot_link = temp_robot;
            managerLog.addLog("exchange distance6 happened");
        }
    }
    if(distance7.size() >= 2)
    {
        double now_dis = distance7[0].first + distance7[1].first;
        double dist1 = getDistance(m_robots[distance7[0].second], m_worktables[m_robots[distance7[1].second].target_table]);
        double dist2 = getDistance(m_robots[distance7[1].second], m_worktables[m_robots[distance7[0].second].target_table]);
        double fur_dis = dist1 + dist2;
        if(now_dis > fur_dis)
        {
            int temp_table = m_robots[distance7[0].second].target_table;
            int temp_robot = m_worktables[m_robots[distance7[0].second].target_table].robot_link;

            m_robots[distance7[0].second].target_table = m_robots[distance7[1].second].target_table;
            m_worktables[m_robots[distance7[0].second].target_table].robot_link = m_worktables[m_robots[distance7[1].second].target_table].robot_link;

            m_robots[distance7[1].second].target_table = temp_table;
            m_worktables[m_robots[distance7[1].second].target_table].robot_link = temp_robot;
        }
    }
}

float Manager::avoid_obs(int8_t robotID)
{
    vector<pair<float, int>> each_rob_dis3;
    vector<pair<float, int>> each_rob_dis2;
    vector<pair<float, int>> each_rob_dis1;
    vector<pair<float, int>> each_rob_dis0;

    vector<pair<float, float>> each_rob_direct3;
    vector<pair<float, float>> each_rob_direct2;
    vector<pair<float, float>> each_rob_direct1;
    vector<pair<float, float>> each_rob_direct0;

    vector<pair<float, float>> each_ang_dif3;
    vector<pair<float, float>> each_ang_dif2;
    vector<pair<float, float>> each_ang_dif1;
    vector<pair<float, float>> each_ang_dif0;

    int distForR2R = 4;                  //重要参数     机器人之间的碰撞检测距离
    int distForFast = 4;
    int distForSlow = 2;
    float ang_big = M_PI/4;              //重要参数     低优先级机器人的旋转方向
    float ang_small = M_PI/6;            //未使用该参数  高优先级机器人的旋转方向
    float ang_range = M_PI/2;            //重要参数     机器人碰撞的检测弧度
    float ang_dif_range = M_PI/6;        //重要参数     机器人角度差值， 当值为0时，表示可退回原来版本
    float w = -1;
    for(int cnt_rob = 0; cnt_rob < 4; cnt_rob++)
    {
        if(cnt_rob == 3)
        {
            for(int i = 0; i < 4; i++)
            {
                if(i != cnt_rob)
                {
                    each_rob_dis3.push_back(make_pair(getDistR2R(m_robots[cnt_rob], m_robots[i]), i));

                    float body_rob_x =  (m_robots[i].loc.x-m_robots[cnt_rob].loc.x)*cos(m_robots[cnt_rob].direct)+(m_robots[i].loc.y-m_robots[cnt_rob].loc.y)*sin(m_robots[cnt_rob].direct);
                    float body_rob_y = -(m_robots[i].loc.x-m_robots[cnt_rob].loc.x)*sin(m_robots[cnt_rob].direct)+(m_robots[i].loc.y-m_robots[cnt_rob].loc.y)*cos(m_robots[cnt_rob].direct);
                    float r_direct1 = atan2(body_rob_y,body_rob_x);
                    each_rob_direct3.push_back(make_pair(getDistR2R(m_robots[cnt_rob], m_robots[i]), r_direct1));
                    
                    float angular_dif = m_robots[cnt_rob].direct - m_robots[i].direct;
                    if(angular_dif > M_PI && angular_dif <= 2*M_PI)
                    {
                        angular_dif = 2*M_PI - angular_dif;
                    }
                    else if(angular_dif < -M_PI && angular_dif >= -2*M_PI)
                    {
                        angular_dif = 2*M_PI + angular_dif;
                    }
                    each_ang_dif3.push_back(make_pair(getDistR2R(m_robots[cnt_rob], m_robots[i]), angular_dif));
                    
                }
            }

            for(int i=0; i<each_rob_dis3.size(); i++)
            {
                for(int j = 0;j < each_rob_dis3.size()-i-1; j++)
                {
                    if(each_rob_dis3[j].first > each_rob_dis3[j+1].first)
                    {
                        pair<float, int> temp;
                        temp = each_rob_dis3[j];
                        each_rob_dis3[j] = each_rob_dis3[j+1];
                        each_rob_dis3[j+1] = temp;
                    }
                }
            }

            for(int i=0; i<each_rob_direct3.size(); i++)
            {
                for(int j = 0;j < each_rob_direct3.size()-i-1; j++)
                {
                    if(each_rob_direct3[j].first > each_rob_direct3[j+1].first)
                    {
                        pair<float, float> temp;
                        temp = each_rob_direct3[j];
                        each_rob_direct3[j] = each_rob_direct3[j+1];
                        each_rob_direct3[j+1] = temp;
                    }
                }
            }

            for(int i=0; i<each_ang_dif3.size(); i++)
            {
                for(int j = 0;j < each_ang_dif3.size()-i-1; j++)
                {
                    if(each_ang_dif3[j].first > each_ang_dif3[j+1].first)
                    {
                        pair<float, float> temp;
                        temp = each_ang_dif3[j];
                        each_ang_dif3[j] = each_ang_dif3[j+1];
                        each_ang_dif3[j+1] = temp;
                    }
                }
            }
        }
        else if(cnt_rob == 2)
        {
            for(int i = 0; i < 4; i++)
            {
                if(i != cnt_rob)
                {
                    each_rob_dis2.push_back(make_pair(getDistR2R(m_robots[cnt_rob], m_robots[i]), i));

                    float body_rob_x =  (m_robots[i].loc.x-m_robots[cnt_rob].loc.x)*cos(m_robots[cnt_rob].direct)+(m_robots[i].loc.y-m_robots[cnt_rob].loc.y)*sin(m_robots[cnt_rob].direct);
                    float body_rob_y = -(m_robots[i].loc.x-m_robots[cnt_rob].loc.x)*sin(m_robots[cnt_rob].direct)+(m_robots[i].loc.y-m_robots[cnt_rob].loc.y)*cos(m_robots[cnt_rob].direct);
                    float r_direct1 = atan2(body_rob_y,body_rob_x);
                    each_rob_direct2.push_back(make_pair(getDistR2R(m_robots[cnt_rob], m_robots[i]), r_direct1));
                    
                    float angular_dif = m_robots[cnt_rob].direct - m_robots[i].direct;
                    
                    if(angular_dif > M_PI && angular_dif <= 2*M_PI)
                    {
                        angular_dif = 2*M_PI - angular_dif;
                    }
                    else if(angular_dif < -M_PI && angular_dif >= -2*M_PI)
                    {
                        angular_dif = 2*M_PI + angular_dif;
                    }
                    each_ang_dif2.push_back(make_pair(getDistR2R(m_robots[cnt_rob], m_robots[i]), angular_dif));
                    
                }
            }

            for(int i=0; i<each_rob_dis2.size(); i++)
            {
                for(int j = 0;j < each_rob_dis2.size()-i-1; j++)
                {
                    if(each_rob_dis2[j].first > each_rob_dis2[j+1].first)
                    {
                        pair<float, int> temp;
                        temp = each_rob_dis2[j];
                        each_rob_dis2[j] = each_rob_dis2[j+1];
                        each_rob_dis2[j+1] = temp;
                    }
                }
            }

            for(int i=0; i<each_rob_direct2.size(); i++)
            {
                for(int j = 0;j < each_rob_direct2.size()-i-1; j++)
                {
                    if(each_rob_direct2[j].first > each_rob_direct2[j+1].first)
                    {
                        pair<float, float> temp;
                        temp = each_rob_direct2[j];
                        each_rob_direct2[j] = each_rob_direct2[j+1];
                        each_rob_direct2[j+1] = temp;
                    }
                }
            }

            for(int i=0; i<each_ang_dif2.size(); i++)
            {
                for(int j = 0;j < each_ang_dif2.size()-i-1; j++)
                {
                    if(each_ang_dif2[j].first > each_ang_dif2[j+1].first)
                    {
                        pair<float, float> temp;
                        temp = each_ang_dif2[j];
                        each_ang_dif2[j] = each_ang_dif2[j+1];
                        each_ang_dif2[j+1] = temp;
                    }
                }
            }
            
        }
        else if(cnt_rob == 1)
        {
            for(int i = 0; i < 4; i++)
            {
                if(i != cnt_rob)
                {
                    each_rob_dis1.push_back(make_pair(getDistR2R(m_robots[cnt_rob], m_robots[i]), i));

                    float body_rob_x =  (m_robots[i].loc.x-m_robots[cnt_rob].loc.x)*cos(m_robots[cnt_rob].direct)+(m_robots[i].loc.y-m_robots[cnt_rob].loc.y)*sin(m_robots[cnt_rob].direct);
                    float body_rob_y = -(m_robots[i].loc.x-m_robots[cnt_rob].loc.x)*sin(m_robots[cnt_rob].direct)+(m_robots[i].loc.y-m_robots[cnt_rob].loc.y)*cos(m_robots[cnt_rob].direct);
                    float r_direct1 = atan2(body_rob_y,body_rob_x);
                    each_rob_direct1.push_back(make_pair(getDistR2R(m_robots[cnt_rob], m_robots[i]), r_direct1));
                    
                    float angular_dif = m_robots[cnt_rob].direct - m_robots[i].direct;
                    if(angular_dif > M_PI && angular_dif <= 2*M_PI)
                    {
                        angular_dif = 2*M_PI - angular_dif;
                    }
                    else if(angular_dif < -M_PI && angular_dif >= -2*M_PI)
                    {
                        angular_dif = 2*M_PI + angular_dif;
                    }
                    each_ang_dif1.push_back(make_pair(getDistR2R(m_robots[cnt_rob], m_robots[i]), angular_dif));
                    
                }
            }

            for(int i=0; i<each_rob_dis1.size(); i++)
            {
                for(int j = 0;j < each_rob_dis1.size()-i-1; j++)
                {
                    if(each_rob_dis1[j].first > each_rob_dis1[j+1].first)
                    {
                        pair<float, int> temp;
                        temp = each_rob_dis1[j];
                        each_rob_dis1[j] = each_rob_dis1[j+1];
                        each_rob_dis1[j+1] = temp;
                    }
                }
            }

            for(int i=0; i<each_rob_direct1.size(); i++)
            {
                for(int j = 0;j < each_rob_direct1.size()-i-1; j++)
                {
                    if(each_rob_direct1[j].first > each_rob_direct1[j+1].first)
                    {
                        pair<float, float> temp;
                        temp = each_rob_direct1[j];
                        each_rob_direct1[j] = each_rob_direct1[j+1];
                        each_rob_direct1[j+1] = temp;
                    }
                }
            }

            for(int i=0; i<each_ang_dif1.size(); i++)
            {
                for(int j = 0;j < each_ang_dif1.size()-i-1; j++)
                {
                    if(each_ang_dif1[j].first > each_ang_dif1[j+1].first)
                    {
                        pair<float, float> temp;
                        temp = each_ang_dif1[j];
                        each_ang_dif1[j] = each_ang_dif1[j+1];
                        each_ang_dif1[j+1] = temp;
                    }
                }
            }
            
        }
        else if(cnt_rob == 0)
        {
            for(int i = 0; i < 4; i++)
            {
                if(i != cnt_rob)
                {
                    each_rob_dis0.push_back(make_pair(getDistR2R(m_robots[cnt_rob], m_robots[i]), i));

                    float body_rob_x =  (m_robots[i].loc.x-m_robots[cnt_rob].loc.x)*cos(m_robots[cnt_rob].direct)+(m_robots[i].loc.y-m_robots[cnt_rob].loc.y)*sin(m_robots[cnt_rob].direct);
                    float body_rob_y = -(m_robots[i].loc.x-m_robots[cnt_rob].loc.x)*sin(m_robots[cnt_rob].direct)+(m_robots[i].loc.y-m_robots[cnt_rob].loc.y)*cos(m_robots[cnt_rob].direct);
                    float r_direct1 = atan2(body_rob_y,body_rob_x);
                    each_rob_direct0.push_back(make_pair(getDistR2R(m_robots[cnt_rob], m_robots[i]), r_direct1));
                    
                    float angular_dif = m_robots[cnt_rob].direct - m_robots[i].direct;
                    if(angular_dif > M_PI && angular_dif <= 2*M_PI)
                    {
                        angular_dif = 2*M_PI - angular_dif;
                    }
                    else if(angular_dif < -M_PI && angular_dif >= -2*M_PI)
                    {
                        angular_dif = 2*M_PI + angular_dif;
                    }
                    each_ang_dif0.push_back(make_pair(getDistR2R(m_robots[cnt_rob], m_robots[i]), angular_dif));
                    
                    
                }
            }

            for(int i=0; i<each_rob_dis0.size(); i++)
            {
                for(int j = 0;j < each_rob_dis0.size()-i-1; j++)
                {
                    if(each_rob_dis0[j].first > each_rob_dis0[j+1].first)
                    {
                        pair<float, int> temp;
                        temp = each_rob_dis0[j];
                        each_rob_dis0[j] = each_rob_dis0[j+1];
                        each_rob_dis0[j+1] = temp;
                    }
                }
            }

            for(int i=0; i<each_rob_direct0.size(); i++)
            {
                for(int j = 0;j < each_rob_direct0.size()-i-1; j++)
                {
                    if(each_rob_direct0[j].first > each_rob_direct0[j+1].first)
                    {
                        pair<float, float> temp;
                        temp = each_rob_direct0[j];
                        each_rob_direct0[j] = each_rob_direct0[j+1];
                        each_rob_direct0[j+1] = temp;
                    }
                }
            }

            for(int i=0; i<each_ang_dif0.size(); i++)
            {
                for(int j = 0;j < each_ang_dif0.size()-i-1; j++)
                {
                    if(each_ang_dif0[j].first > each_ang_dif0[j+1].first)
                    {
                        pair<float, float> temp;
                        temp = each_ang_dif0[j];
                        each_ang_dif0[j] = each_ang_dif0[j+1];
                        each_ang_dif0[j+1] = temp;
                    }
                }
            }
            
        }
    }
    if(robotID == 0)
    {
        for(int i=0; i<each_rob_dis0.size(); i++)
        {
            if(each_rob_direct0[i].first <= distForR2R &&
             (each_rob_direct0[i].second >= -ang_range && each_rob_direct0[i].second <= ang_range) )
            {
                if(robotID < each_rob_dis0[i].second)
                {
                    if((each_rob_direct0[i].second >= 0 && each_rob_direct0[i].second <= ang_range) && ( !(each_ang_dif0[i].second >= 0 && each_ang_dif0[i].second <= ang_dif_range) ))
                    {
                        w = -ang_big;
                    }
                    else if((each_rob_direct0[i].second >= -ang_range && each_rob_direct0[i].second < 0) &&( !(each_ang_dif0[i].second >= -ang_dif_range && each_ang_dif0[i].second <= 0) ))
                    {
                        w = ang_big;
                    }
                }
                #if react
                else
                {
                    if(each_rob_direct0[i].second >= 0 && each_rob_direct0[i].second <= ang_range)
                    {
                        w = -ang_small;
                    }
                    else
                    {
                        w = ang_small;
                    }
                }
                #endif
                w = 10*w;
                break;
            }
        }
    }
    else if(robotID == 1)
    {
        for(int i=0; i<each_rob_dis1.size(); i++)
        {
            if(each_rob_direct1[i].first <= distForR2R &&
             (each_rob_direct1[i].second >= -ang_range && each_rob_direct1[i].second <= ang_range) )
            {
                if(robotID < each_rob_dis1[i].second)
                {
                    if((each_rob_direct1[i].second >= 0 && each_rob_direct1[i].second <= ang_range) && ( !(each_ang_dif1[i].second >= 0 && each_ang_dif1[i].second <= ang_dif_range) ))
                    {
                        w = -ang_big;
                    }
                    else if((each_rob_direct1[i].second >= -ang_range && each_rob_direct1[i].second < 0) &&( !(each_ang_dif1[i].second >= -ang_dif_range && each_ang_dif1[i].second <= 0) ))
                    {
                        w = ang_big;
                    }
                }
                #if react
                else
                {
                    if(each_rob_direct1[i].second >= 0 && each_rob_direct1[i].second <= ang_range)
                    {
                        w = -ang_small;
                    }
                    else
                    {
                        w = ang_small;
                    }
                }
                #endif
                w = 10*w;
                break;
            }
        }
    }
    else if(robotID == 2)
    {
        for(int i=0; i<each_rob_dis2.size(); i++)
        {
            if(each_rob_direct2[i].first <= distForR2R &&
             (each_rob_direct2[i].second >= -ang_range && each_rob_direct2[i].second <= ang_range) )
            {
                if(robotID < each_rob_dis2[i].second)
                {
                    if((each_rob_direct2[i].second >= 0 && each_rob_direct2[i].second <= ang_range) && ( !(each_ang_dif2[i].second >= 0 && each_ang_dif2[i].second <= ang_dif_range) ))
                    {
                        w = -ang_big;
                    }
                    else if((each_rob_direct2[i].second >= -ang_range && each_rob_direct2[i].second < 0) &&( !(each_ang_dif2[i].second >= -ang_dif_range && each_ang_dif2[i].second <= 0) ))
                    {
                        w = ang_big;
                    }
                }
                #if react
                else
                {
                    if(each_rob_direct2[i].second >= 0 && each_rob_direct2[i].second <= ang_range)
                    {
                        w = -ang_small;
                    }
                    else
                    {
                        w = ang_small;
                    }
                }
                #endif
                w = 10*w;
                break;
            }
        }
    }
    else if(robotID == 3)
    {
        for(int i=0; i<each_rob_dis3.size(); i++)
        {
            if(each_rob_direct3[i].first <= distForR2R &&
             (each_rob_direct3[i].second >= -ang_range && each_rob_direct3[i].second <= ang_range) )
            {
                if(robotID < each_rob_dis3[i].second)
                {
                    if(robotID < each_rob_dis2[i].second)
                {
                    if((each_rob_direct3[i].second >= 0 && each_rob_direct3[i].second <= ang_range) && ( !(each_ang_dif3[i].second >= 0 && each_ang_dif3[i].second <= ang_dif_range) ))
                    {
                        w = -ang_big;
                    }
                    else if((each_rob_direct3[i].second >= -ang_range && each_rob_direct3[i].second < 0) &&( !(each_ang_dif3[i].second >= -ang_dif_range && each_ang_dif3[i].second <= 0) ))
                    {
                        w = ang_big;
                    }
                }
                }
                #if react
                else
                {
                    if(each_rob_direct3[i].second >= 0 && each_rob_direct3[i].second <= ang_range)
                    {
                        w = -ang_small;
                    }
                    else
                    {
                        w = ang_small;
                    }
                }
                #endif
                w = 10*w;
                break;
            }
        }
    }
    return w;
}


#define setPoint1 1


#if setPoint1

bool Manager::set_Pointv4(uint8_t robotID,double ref_x,double ref_y){
    static float last_refx=ref_x;
    static float last_refy=ref_y;


    if (last_refx != ref_x || last_refy != ref_y){
        m_robots[robotID].arrived= false;
    }
    float r_direct=0;
    float body_refx=(ref_x-m_robots[robotID].loc.x)*cos(m_robots[robotID].direct)+(ref_y-m_robots[robotID].loc.y)*sin(m_robots[robotID].direct);
    float body_refy = -(ref_x-m_robots[robotID].loc.x)*sin(m_robots[robotID].direct)+(ref_y-m_robots[robotID].loc.y)*cos(m_robots[robotID].direct);
//    if (abs(body_refy)<0.0001){//防止奇异跳变
//        r_direct=0;
//    }
//    else
//    {
    r_direct=atan2(body_refy,body_refx);
    //}
    float v=6.0f;
    float distance = sqrt((ref_y-m_robots[robotID].loc.y)*(ref_y-m_robots[robotID].loc.y)+(ref_x-m_robots[robotID].loc.x)*(ref_x-m_robots[robotID].loc.x));

    if(distance<=1) v=1;
    else v = 6;
    if (v>=6) v=6;
    if (v<=-2) v=-2;
    float w=r_direct;

    if(abs(w)>=M_PI_4) v=2;

    int r=0;
    int obj=m_robots->objectID-'0';
    if(obj>0) r=0.53;
    else r=0.45;

    w = 10*w;
    if (w>=M_PI) w=M_PI;
    if (w<-M_PI) w=-M_PI;
    /*
    if(abs(r_direct)>=M_PI_4) v=-1;
    if(distance<=6 && abs(r_direct)>M_PI*2) v=-2;
    */
    
    float ang = avoid_obs(robotID);
    if(ang != -1 && ang != -10)
        w = ang;

    //在靠墙或者目标点附近时，转弯要减速
//    if((x_A<3 || x_A>47 || y_A<3 || y_A>47) || (distance<2)){
//        if (distance>=1) v=6.0f;
//        else if (distance>=0.5) v=4.0f;
//        else if (distance<=0.5)v=1.0f;
//        if(w>=M_PI/2 || w<=-M_PI/2) v=1;//通过调整这个可以调整圈的大小
//        else if (w>=M_PI/4 || w<=-M_PI/4) v=2;
//        else v=4;
//    }
    //已经贴墙
//    if(m_robots[robotID].loc.x>=50-r || m_robots[robotID].loc.x<=r ||
//       m_robots[robotID].loc.y>=50-r || m_robots[robotID].loc.y<=r)
//        v=-2.0f;
    //已经到达
    if (m_robots[robotID].arrived)
    {
        v=0.0f;
    }
    last_refx=ref_x;
    last_refy=ref_y;
    control(robotID,ACT_FORWARD,v);
    control(robotID,ACT_ROTATE,w);
    if (m_robots[robotID].worktableID==m_robots[robotID].target_table){
        m_robots[robotID].arrived= true;
    }
    else {
        m_robots[robotID].arrived=false;
    }
    return m_robots[robotID].arrived;



}
#endif

#if setPoint2
bool Manager::set_Pointv4(uint8_t robotID,double ref_x,double ref_y){
    static float last_refx=ref_x;
    static float last_refy=ref_y;
    if (last_refx != ref_x || last_refy != ref_y){
        m_robots[robotID].arrived= false;
    }
    float r_direct=0;
    float body_refx=(ref_x-m_robots[robotID].loc.x)*cos(m_robots[robotID].direct)+(ref_y-m_robots[robotID].loc.y)*sin(m_robots[robotID].direct);
    float body_refy = -(ref_x-m_robots[robotID].loc.x)*sin(m_robots[robotID].direct)+(ref_y-m_robots[robotID].loc.y)*cos(m_robots[robotID].direct);
//    if (abs(body_refy)<0.0001){//防止奇异跳变
//        r_direct=0;
//    }
//    else
//    {
    r_direct=atan2(body_refy,body_refx);
    //}
    float v=6.0f;
    float distance = sqrt((ref_y-m_robots[robotID].loc.y)*(ref_y-m_robots[robotID].loc.y)+(ref_x-m_robots[robotID].loc.x)*(ref_x-m_robots[robotID].loc.x));

    if (v>=6) v=6;
    if (v<=-2) v=-2;
    float w=r_direct;

    int r=0;
    int obj=m_robots->objectID-'0';
    if(obj>0) r=0.53;
    else r=0.45;

    //右侧墙壁
    if (m_robots[robotID].loc.x>=49 && m_robots[robotID].direct >= -M_PI/4 && m_robots[robotID].direct <= M_PI/4){
        v=3.0f;
    }
    //左侧墙壁
    if (m_robots[robotID].loc.x<=1 && m_robots[robotID].direct <= -3*M_PI/4 && m_robots[robotID].direct >= 3*M_PI/4){
        v=3.0f;
    }
    //上面墙壁
    if (m_robots[robotID].loc.y>=49 && m_robots[robotID].direct >= M_PI/4 && m_robots[robotID].direct <= 3*M_PI/4){
        v=3.0f;
    }
    //下面墙壁
    if (m_robots[robotID].loc.y<=1 && m_robots[robotID].direct >= -3*M_PI/4 && m_robots[robotID].direct <= -M_PI/4){
        v=3.0f;
    }


    //避开前方障碍物
    double x_A=m_robots[robotID].loc.x;//A机器人的横坐标
    double y_A=m_robots[robotID].loc.y;//A机器人的纵坐标
    float A_direct = m_robots[robotID].direct;//A的朝向
    for (int i = 0; i < 4; ++i) {
        if(i!=robotID){
            double x_B,y_B;//B机器人的坐标
            x_B = m_robots[i].loc.x;
            y_B = m_robots[i].loc.y;
            double AB;//机器人间的距离
            //计算两个机器人间的距离
            AB = sqrt((x_A-x_B)*(x_A-x_B) + (y_A-y_B)*(y_A-y_B));
            float ang_A;//A、B机器人的连线与A朝向的夹角
            float B_direct = m_robots[i].direct;//B的朝向
            if(AB<5){
                float temp_angle;//AB与地图横坐标正方向的夹角
                if(y_B>y_A){//B在A的上方
                    temp_angle = acos((x_B-x_A)/AB);
                }
                else if(y_B<y_A){//B在A下方
                    temp_angle = -acos((x_B-x_A)/AB);
                }
                else{
                    if(x_A>x_B) temp_angle = M_PI;//B在A左边
                    else temp_angle = 0;//B在A右边
                }
                //判断B是否在A的扇形区域
                //便宜的给贵的让路，一样价格的还没想好
                ang_A = temp_angle - A_direct;//大于0，在左前方
                int obj_i = m_robots[i].objectID-'0';
                float ang = A_direct - B_direct;//理论上他的范围[-M_PI/2,M_PI*3/2]
                if((AB<8 & abs(ang_A)<=M_PI/2 & abs(ang_A)<asin(1.1/AB)) && AB>2.5 ){//矩形区域
                    if(obj_i>=obj){
                        if(ang>=0 & ang<M_PI-abs(ang_A)-asin(1.2/AB)){
                            w = asin(1.2/AB)+ang_A;
                        }else if(ang>=M_PI-abs(ang_A)-asin(1.2/AB) & ang<=M_PI+abs(ang_A)+asin(1.2/AB)){
                            if(ang_A>=0) w = -asin(1.2/AB)+ang_A;
                            else w = asin(1.2/AB)+ang_A;
                        }else{
                            w = -asin(1.2/AB)+ang_A;
                        }
                    }
                } else if ((AB<=2.5 & abs(ang_A)<=M_PI/2)){
                    double r_i=obj_i>0?0.53:0.45;
                    //不准并排行使
                    if(AB<r+r_i+0.5 && abs(ang_A)>M_PI/2-M_PI/18 && abs(ang)<M_PI/6){
                        if (m_robots[i].v_line.x+m_robots[i].v_line.y>0)
                            if(obj_i>=obj) v = -2;
                    }
                    if(obj_i>=obj){
                        if(ang_A>=0) w = -M_PI/2;
                        else w = M_PI/2;
                        if(AB<=r+r_i){
                            v=-2;//撞了就退一下
                        }
                    }
                }
            }
        }
    }
    w = 20*w;
    if (w>=M_PI) w=M_PI;
    if (w<-M_PI) w=-M_PI;
    if(w>M_PI/2 || w<-M_PI/2) v=5;

    //在靠墙或者目标点附近时，转弯要减速
    if((x_A<3 || x_A>47 || y_A<3 || y_A>47) || (distance<2)){
        if (distance>=1) v=6.0f;
        else if (distance>=0.5) v=4.0f;
        else if (distance<=0.5)v=1.0f;
        if(w>=M_PI/2 || w<=-M_PI/2) v=1;//通过调整这个可以调整圈的大小
        else if (w>=M_PI/4 || w<=-M_PI/4) v=2;
        else v=4;
    }
    //已经贴墙
    if(m_robots[robotID].loc.x>=50-r || m_robots[robotID].loc.x<=r ||
       m_robots[robotID].loc.y>=50-r || m_robots[robotID].loc.y<=r)
        v=-2.0f;
    //已经到达
    if (m_robots[robotID].arrived)
    {
        v=0.0f;
    }
    last_refx=ref_x;
    last_refy=ref_y;
    control(robotID,ACT_FORWARD,v);
    control(robotID,ACT_ROTATE,w);
    if (m_robots[robotID].worktableID==m_robots[robotID].target_table){
        m_robots[robotID].arrived= true;
    }
    else {
        m_robots[robotID].arrived=false;
    }
    return m_robots[robotID].arrived;

}
#endif

bool readUntilOK(vector<string> &strings) {
    char line[1024];
    strings.clear();
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        } 
        
        strings.push_back(string(line));
    }
    return false;
}



//su


#endif
