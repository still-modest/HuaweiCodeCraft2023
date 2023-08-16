/*修改了较多参数*/
#ifndef __SYSTEM_H
#define __SYSTEM_H

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <stdio.h>
#include <algorithm>
#include <iomanip>
using namespace std;



/******************** 各种声明 ********************/
enum actions{ // 机器人的行动
    FORWARD_ACTION,    // 前进（米/秒）[-2, 6]
    ROTATE_ACTION,     // 旋转速度（弧度/秒）[-π,π]，正值表示逆时针旋转
    BUY_ACTION,        // 购买当前工作台的物品
    SELL_ACTION,       // 出售当前物品给当前工作台
    DESTROY_ACTION     // 摧毁物品
};

struct Robot{   // 机器人属性
    uint8_t ID; // 机器人序号
    char workBenchID=-1; // 所处工作台ID，-1表示没有
    char objectID=0;  // 携带物品ID，0表示没有，[1,7]为对应物品
    float value_time=0; // 时间价值系数[0.8, 1]，0表示无物品
    float value_crash=0;    // 碰撞价值系数[0.8, 1]，0表示无物品
    float vel_rad=0;  // 角速度（弧度/秒），正数表示顺时针
    struct {  // 线速度（米/秒）
        double x;
        double y;
    }vel_line;
    double direct=0;   // 朝向[-π,π]；0，右方向；π/2，上方向
    struct{     // 坐标
        double x;
        double y;
    }loc;

    char order = 0; //任务 0没任务 1买任务 2卖任务
    vector<pair<double,uint8_t> > distances;    //机器人与工作台距离
    bool arrived_1=false;
    bool arrived_2=false;
    bool arrived = false;
    int target_table=-1;
    vector<int> v_target_tables={-1,-1};
    bool busy= false;
    bool no_chice=false;



};

struct Worktable{   // 工作台属性
    uint8_t ID; // 序号
    uint8_t classID;    // 类别[1,9]
    int last=-1;   // 剩余生产时间(帧数)；-1，没有生产；0，生产因输出格满而阻塞；>=0，剩余生产帧数
    uint16_t sta_component=0; // 原材料格状态，二进制表示；如 48(000110000) 表示拥有物品4和5。
    uint8_t sta_produce=0; // 产品格状态；0，无；1，有
    struct{     // 坐标
        double x;
        double y;
    } loc;
    vector<int> robot={0,0,0,0};//表示未与任何机器人配对
    int robot_link=4;
};

class System{
public:
    /**
     * @brief 解析地图信息以初始化
     *
     * @param mapStr 接收到的地图信息
     */
    System(vector<string> mapStr);

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
         * @brief 机器人避障碍
         * @param robotID 机器人ID
         * @return 机器人旋转角度
         */
    float avoid_obs(int8_t robotID);

    /**
     * @brief 机器人与工作台距离
     *
     */
    void robotTableDistance();


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
    void statis_need();
    void last_process();
    vector<int> loop_4(int robot_id);
    vector<int> loop_3(int robot_id);
    vector<vector<int>> loop_2(int robot_id);
    vector<int> scan_mostneed(int robot_id);
    vector<int> cost_fun(vector<pair<double,vector<int>>> dis_all, int robot_id);
    void robot_actionv2();
    double cal_lost(int fps);
    bool check_robotlink(int cnt_table, int buy_sell,int sell_class);
    bool check_willhave(int cnt_table,int object);
    bool avoid_area(int robotID1, int robotID2, float distForR2R);
    float avoid_obs2(int robotID);
    int adiust_dynamic();
    int check_linknum(int cnt_table);
    Robot glo_robots[5];  // 四台机器人
    vector<Worktable> glo_workBenches; // 工作台
    vector<int> table_norobot={0,0,0,0};
    
    int map_id=0;
    int stop_time=0;

    int glo_frameID; //帧序号
    uint32_t glo_money;   // 当前金钱数
    uint8_t glo_count_workBench; // 工作台数量

    int class_1_enough=0;
    int class_2_enough=0;
    int class_3_enough=0;
    int class_4_enough=0;
    int class_5_enough=0;
    int class_6_enough=0;

    int need_4=0;
    int need_5=0;
    int need_6=0;

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
System::System(vector<string> mapStr){
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
                glo_robots[cnt_robot].ID = cnt_robot;
                glo_robots[cnt_robot].loc.x = float(x)*0.5 + 0.25;
                glo_robots[cnt_robot].loc.y = float(99-l)*0.5 + 0.25;
                cnt_robot++;
            }else if((pt >= '1') && (pt <= '9')){   // 工作台
                Worktable wTable;
                wTable.ID = cnt_wTable;
                wTable.classID = pt - '0';
                wTable.loc.x = float(x)*0.5 + 0.25;
                wTable.loc.y = float(99-l)*0.5 + 0.25;
                glo_workBenches.push_back(wTable);
                cnt_wTable++;

                glo_count_workBench = cnt_wTable;
            }
        }
    }
}


void System::praseFrameStr(vector<string> frameStr){
    istringstream one_line;	// 输入流
    string obj;

    // 1行-帧序号，金钱数
    one_line.str(frameStr[0]);
    one_line >> obj;
    glo_frameID = stoul(obj);
    one_line >> obj;
    glo_money = stoul(obj);

    // 2行-工作台数量(跳过)
    // 3~3+K行-工作台信息
    for(int i=0; i<glo_count_workBench; i++){
        one_line.str(frameStr[i+2]);
        one_line >> obj;
        // glo_workBenches[i].classID = obj[0] - '0';
        one_line >> obj;
        // glo_workBenches[i].loc.x = stod(obj);
        one_line >> obj;
        // glo_workBenches[i].loc.y = stod(obj);
        one_line >> obj;
        glo_workBenches[i].last = stoi(obj);
        one_line >> obj;
        glo_workBenches[i].sta_component = stoi(obj);
        one_line >> obj;
        glo_workBenches[i].sta_produce = stoul(obj);
    }

    // 剩余4行-机器人信息
    for(int i=0; i<4; i++){
        one_line.str(frameStr[i+2+glo_count_workBench]);
        one_line >> obj;
        glo_robots[i].workBenchID = stoi(obj);
        one_line >> obj;
        glo_robots[i].objectID = stoi(obj);
        one_line >> obj;
        glo_robots[i].value_time = stof(obj);
        one_line >> obj;
        glo_robots[i].value_crash = stof(obj);
        one_line >> obj;
        glo_robots[i].vel_rad = stod(obj);
        one_line >> obj;
        glo_robots[i].vel_line.x = stod(obj);
        one_line >> obj;
        glo_robots[i].vel_line.y = stod(obj);
        one_line >> obj;
        glo_robots[i].direct = stod(obj);
        one_line >> obj;
        glo_robots[i].loc.x = stod(obj);
        one_line >> obj;
        glo_robots[i].loc.y = stod(obj);
    }
}

void System::control(uint8_t robotID, actions action, float intensity){
    char str[100];
    switch(action){
        case FORWARD_ACTION:
            sprintf(str, "forward %d %f\n", robotID, intensity);
            break;
        case ROTATE_ACTION:
            sprintf(str, "rotate %d %f\n", robotID, intensity);
            break;
        case BUY_ACTION:
            sprintf(str, "buy %d\n", robotID);
            break;
        case SELL_ACTION:
            sprintf(str, "sell %d\n", robotID);
            break;
        case DESTROY_ACTION:
            sprintf(str, "destroy %d\n", robotID);
            break;
        default:
            return;
    }
    m_ctlStr.push_back(string(str));    // 输入到控制指令列表中
}

void System::doActions(){
    printf("%d\n", glo_frameID);    // 发送帧ID

    while(!m_ctlStr.empty()){
        cout << m_ctlStr.back();
        m_ctlStr.pop_back();
    }

    printf("OK\n"); // 报文尾
    fflush(stdout);
}

/**********获取距离**********/
double System::getDistance(Robot m_robot, Worktable m_worktable)
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

/**********获取所有机器人与所有工作台距离**********/
void System::robotTableDistance()
{

    for(int i = 0; i < 4; i++)
    {
        glo_robots[i].distances.clear();
        for(int j = 0; j < glo_workBenches.size(); j++)
        {
            double dis = getDistance(glo_robots[i], glo_workBenches[j]); //获取机器人与每个工作台的距离，存入容器distances中
            uint8_t tableID = glo_workBenches[j].classID;

            glo_robots[i].distances.push_back(make_pair(dis, tableID));
            /*if(glo_robots[i].distances[j].first < 0.4)
            {
                glo_robots[i].workBenchID = j;
            }*/
        }
    }
}

/**********获取机器人之间的距离**********/
double System::getDistR2R(Robot m_robot1, Robot m_robot2)
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


/**********依次由最大的工作台到最小的工作台与四个机器人进行排序**********/
vector<pair<uint8_t, Worktable>> System::distanceSort()
{
    vector<pair<uint8_t, Worktable>> vp_sort;

    robotTableDistance();
    for(int i = 0; i < glo_workBenches.size(); i++)
    {
        uint8_t robotId = 0;
        double dist= 100;
        for(int j = 0; j < 4; j++)
        {
            if(glo_robots[j].distances[i].first < dist)
            {
                robotId = j;
                dist = glo_robots[j].distances[i].first;
                //vp_sort.push_back(make_pair(robotId, glo_workBenches[i]));

            }
        }
        vp_sort.push_back(make_pair(robotId, glo_workBenches[i]));
    }


    for (int  i = 0; i < glo_workBenches.size(); i++)
    {
        for(int j = 0; j < glo_workBenches.size()-i-1;j++)
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


    return vp_sort;
}

void System::statis_table()
{
    int num7=0;
    for(int i=0;i<glo_count_workBench;i++)
    {
        if(glo_workBenches[i].classID==7) num7++;
    }
    if(num7==8) map_id=1;
    if(num7==2) map_id=2;
    if(num7==0) map_id=3;
    if(num7==1) map_id=4;

    if(map_id==1) stop_time=8850;
    if(map_id==2) stop_time=8850;
    if(map_id==4) stop_time=8650;
    if(map_id==3) stop_time=8850;

}
void System::statis_need()
{

}
void System::statis_enough()
{
    int table_class1=0;
    int table_class2=0;
    int table_class3=0;
    int table_class4=0;
    int table_class5=0;
    int table_class6=0;
    for(int i=0;i<glo_count_workBench;i++)
    {
        if(glo_workBenches[i].classID==4)
        {
            if((glo_workBenches[i].sta_component & (1<<1))==0) table_class1++;
            if((glo_workBenches[i].sta_component & (1<<2))==0) table_class2++;
        }
        if(glo_workBenches[i].classID==5)
        {
            if((glo_workBenches[i].sta_component & (1<<1))==0) table_class1++;
            if((glo_workBenches[i].sta_component & (1<<3))==0) table_class3++;
        }
        if(glo_workBenches[i].classID==6)
        {
            if((glo_workBenches[i].sta_component & (1<<2))==0) table_class2++;
            if((glo_workBenches[i].sta_component & (1<<3))==0) table_class3++;
        }
        if(no_7)
        {
            table_class4=100;
            table_class5=100;
            table_class6=100;
        }
        else
        {
            if(glo_workBenches[i].classID==7)
            {
                if((glo_workBenches[i].sta_component & (1<<4))==0) table_class4++;
                if((glo_workBenches[i].sta_component & (1<<5))==0) table_class5++;
                if((glo_workBenches[i].sta_component & (1<<6))==0) table_class6++;
            }
        }

    }
    for(int i=0;i<4;i++)
    {
//        if(glo_robots[i].objectID==0)
//        {
//            if(glo_workBenches[glo_robots[i].target_table].classID==1)//即将去买1
//            {
//                table_class1--;
//            }
//            if(glo_workBenches[glo_robots[i].target_table].classID==2)//即将去买2
//            {
//                table_class2--;
//            }
//            if(glo_workBenches[glo_robots[i].target_table].classID==3)//即将去买3
//            {
//                table_class3--;
//            }
//            if(glo_workBenches[glo_robots[i].target_table].classID==4)//即将去买4
//            {
//                table_class4--;
//            }
//            if(glo_workBenches[glo_robots[i].target_table].classID==5)//即将去买5
//            {
//                table_class5--;
//            }
//            if(glo_workBenches[glo_robots[i].target_table].classID==6)//即将去买6
//            {
//                table_class6--;
//            }
//        }
        if(glo_robots[i].objectID!=0)
        {
            if(glo_robots[i].objectID==1) table_class1--;
            if(glo_robots[i].objectID==2) table_class2--;
            if(glo_robots[i].objectID==3) table_class3--;
            if(glo_robots[i].objectID==4) table_class4--;
            if(glo_robots[i].objectID==5) table_class5--;
            if(glo_robots[i].objectID==6) table_class6--;
        }
    }
    need_4 = table_class4;
    need_5=table_class5;
    need_6=table_class6;
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


}
bool System::check_robotlink(int cnt_table, int buy_sell,int sell_class) {
    if(glo_workBenches[cnt_table].robot==table_norobot) return true;
    if(glo_workBenches[cnt_table].classID<=3) return true;
    for(int i=0;i<4;i++)
    {

        if(buy_sell==1)//查看是否已绑定来买的机器人
        {
            if(glo_workBenches[cnt_table].robot[i]==1) return false;
        }
        if(glo_workBenches[cnt_table].classID >3 && glo_workBenches[cnt_table].classID<=7)
        {
            if(buy_sell==2)//查看送来卖的情况
            {
                if(glo_workBenches[cnt_table].robot[i]==2)
                {
                    if(glo_robots[i].arrived_1)
                    {
                        if(glo_robots[i].objectID==sell_class) return false;
                    }
                    if(!glo_robots[i].arrived_1)
                    {
                        if(glo_workBenches[glo_robots[i].v_target_tables[0]].classID==sell_class) return false;
                    }
                }
            }
        }

    }
    return true;
}
int System::check_linknum(int cnt_table) {
    int ans=0;
    for(int i=0;i<4;i++)
    {
        if(glo_workBenches[cnt_table].robot[i]==1) ans++;
    }
    return ans;
}
bool System::check_willhave(int cnt_table, int object) {
    for(int i=0;i<4;i++)
    {
        if(glo_workBenches[cnt_table].robot[i]==2)
        {
            if(glo_robots[i].arrived_1)
            {
                if(glo_robots[i].objectID==object) return true;
            }
            if(!glo_robots[i].arrived_1)
            {
                if(glo_workBenches[glo_robots[i].v_target_tables[0]].classID==object) return true;
            }
        }
    }
    return false;
}
bool MyPaircmp(pair<double,int> a, pair<double,int> b)
{
    return a.first<b.first;
}
bool MyPaircmp1(pair<double,vector<int>> a, pair<double,vector<int>> b)
{
    return a.first<b.first;
}
bool MyPaircmp2(pair<double,vector<int>> a, pair<double,vector<int>> b)
{
    return a.first>b.first;
}
vector<int> System::scan_mostneed(int robot_id)
{
    vector<pair<double,vector<int>>> dis_all;
    vector<int> no_choice;
    no_choice.push_back(1);
    no_choice.push_back(2);
    for(int i=0;i<glo_count_workBench;i++)
    {
        if(glo_workBenches[i].classID==9)
        {
            for(int j=0;j<glo_count_workBench;j++)
            {
                if(glo_workBenches[j].classID==1 && (glo_workBenches[j].sta_produce ))
                {
                    vector<int> table_temp;
                    table_temp.push_back(j);
                    table_temp.push_back(i);
                    double dis_temp=sqrt((glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x) *
                                         (glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x)
                                         + (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y) *
                                           (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y))+
                                    sqrt((glo_workBenches[i].loc.x - glo_workBenches[j].loc.x) *
                                         (glo_workBenches[i].loc.x - glo_workBenches[j].loc.x)
                                         + (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y) *
                                           (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y));
                    dis_all.push_back(make_pair(dis_temp,table_temp));
                }
                if(glo_workBenches[j].classID==2 && (glo_workBenches[j].sta_produce ))
                {
                    vector<int> table_temp;
                    table_temp.push_back(j);
                    table_temp.push_back(i);
                    double dis_temp=sqrt((glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x) *
                                         (glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x)
                                         + (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y) *
                                           (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y))+
                                    sqrt((glo_workBenches[i].loc.x - glo_workBenches[j].loc.x) *
                                         (glo_workBenches[i].loc.x - glo_workBenches[j].loc.x)
                                         + (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y) *
                                           (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y));
                    dis_all.push_back(make_pair(dis_temp,table_temp));
                }
                if(glo_workBenches[j].classID==3 && (glo_workBenches[j].sta_produce) )
                {
                    vector<int> table_temp;
                    table_temp.push_back(j);
                    table_temp.push_back(i);
                    double dis_temp=sqrt((glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x) *
                                         (glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x)
                                         + (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y) *
                                           (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y))+
                                    sqrt((glo_workBenches[i].loc.x - glo_workBenches[j].loc.x) *
                                         (glo_workBenches[i].loc.x - glo_workBenches[j].loc.x)
                                         + (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y) *
                                           (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y));
                    dis_all.push_back(make_pair(dis_temp,table_temp));
                }
                if(glo_workBenches[j].classID==4 && map_id!=1 && (( (glo_workBenches[j].sta_produce || (glo_workBenches[j].last<=30 &&glo_workBenches[j].last>0))) )&& check_robotlink(j,1,0))
                {
                    vector<int> table_temp;
                    table_temp.push_back(j);
                    table_temp.push_back(i);
                    double dis_temp=sqrt((glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x) *
                                         (glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x)
                                         + (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y) *
                                           (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y))+
                                    sqrt((glo_workBenches[i].loc.x - glo_workBenches[j].loc.x) *
                                         (glo_workBenches[i].loc.x - glo_workBenches[j].loc.x)
                                         + (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y) *
                                           (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y));
                    dis_all.push_back(make_pair(dis_temp,table_temp));
                }
                if(glo_workBenches[j].classID==5 && map_id!=1 && (( (glo_workBenches[j].sta_produce || (glo_workBenches[j].last<=30 &&glo_workBenches[j].last>0))))&& check_robotlink(j,1,0))
                {
                    vector<int> table_temp;
                    table_temp.push_back(j);
                    table_temp.push_back(i);
                    double dis_temp=sqrt((glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x) *
                                         (glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x)
                                         + (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y) *
                                           (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y))+
                                    sqrt((glo_workBenches[i].loc.x - glo_workBenches[j].loc.x) *
                                         (glo_workBenches[i].loc.x - glo_workBenches[j].loc.x)
                                         + (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y) *
                                           (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y));
                    dis_all.push_back(make_pair(dis_temp,table_temp));
                }
                if(glo_workBenches[j].classID==6 && map_id!=1 && (( (glo_workBenches[j].sta_produce || (glo_workBenches[j].last<=30 &&glo_workBenches[j].last>0))) )&& check_robotlink(j,1,0))
                {
                    vector<int> table_temp;
                    table_temp.push_back(j);
                    table_temp.push_back(i);
                    double dis_temp=sqrt((glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x) *
                                         (glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x)
                                         + (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y) *
                                           (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y))+
                                    sqrt((glo_workBenches[i].loc.x - glo_workBenches[j].loc.x) *
                                         (glo_workBenches[i].loc.x - glo_workBenches[j].loc.x)
                                         + (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y) *
                                           (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y));
                    dis_all.push_back(make_pair(dis_temp,table_temp));
                }
            }
        }
        if(glo_workBenches[i].classID==8)
        {
            for(int j=0;j<glo_count_workBench;j++)
            {
                if(glo_workBenches[j].classID==7 && (glo_workBenches[j].sta_produce || (glo_workBenches[j].last<=50 &&glo_workBenches[j].last>0)) && glo_workBenches[j].robot==table_norobot && glo_frameID<=8250)
                {
                    vector<int> table_temp;
                    table_temp.push_back(j);
                    table_temp.push_back(i);
                    double dis_temp=sqrt((glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x) *
                                         (glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x)
                                         + (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y) *
                                           (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y))+
                                    sqrt((glo_workBenches[i].loc.x - glo_workBenches[j].loc.x) *
                                         (glo_workBenches[i].loc.x - glo_workBenches[j].loc.x)
                                         + (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y) *
                                           (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y));
                    dis_all.push_back(make_pair(dis_temp,table_temp));
                }
                if(glo_workBenches[j].classID==7 && (glo_workBenches[j].sta_produce || (glo_workBenches[j].last<=100 &&glo_workBenches[j].last>0)) &&
                        check_robotlink(j,1,0) && glo_frameID>8250)
                {
                    vector<int> table_temp;
                    table_temp.push_back(j);
                    table_temp.push_back(i);
                    double dis_temp=sqrt((glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x) *
                                         (glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x)
                                         + (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y) *
                                           (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y))+
                                    sqrt((glo_workBenches[i].loc.x - glo_workBenches[j].loc.x) *
                                         (glo_workBenches[i].loc.x - glo_workBenches[j].loc.x)
                                         + (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y) *
                                           (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y));
                    dis_all.push_back(make_pair(dis_temp,table_temp));
                }
            }
        }
        if(glo_workBenches[i].classID==7)
        {
            if(glo_workBenches[i].sta_component==112) continue;
            if(((glo_workBenches[i].sta_component & (1<<4))==0) && check_robotlink(i,2,4))
            {
                for(int j=0;j<glo_count_workBench;j++)
                {
                    if(glo_workBenches[j].classID==4 && (( (glo_workBenches[j].sta_produce || (glo_workBenches[j].last<=30 &&glo_workBenches[j].last>0)))) && check_robotlink(j,1,0))
                    {
                        vector<int> table_temp;
                        table_temp.push_back(j);
                        table_temp.push_back(i);
                        double dis_temp=sqrt((glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x) *
                                             (glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x)
                                             + (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y) *
                                               (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y))+
                                        sqrt((glo_workBenches[i].loc.x - glo_workBenches[j].loc.x) *
                                             (glo_workBenches[i].loc.x - glo_workBenches[j].loc.x)
                                             + (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y) *
                                               (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y));
                        dis_all.push_back(make_pair(dis_temp,table_temp));
                    }
                }
            }
            if(((glo_workBenches[i].sta_component & (1<<5))==0) && check_robotlink(i,2,5))
            {
                for(int j=0;j<glo_count_workBench;j++)
                {
                    if(glo_workBenches[j].classID==5 && ( (glo_workBenches[j].sta_produce || (glo_workBenches[j].last<=30 &&glo_workBenches[j].last>0))) && check_robotlink(j,1,0))
                    {
                        vector<int> table_temp;
                        table_temp.push_back(j);
                        table_temp.push_back(i);
                        double dis_temp=sqrt((glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x) *
                                             (glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x)
                                             + (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y) *
                                               (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y))+
                                        sqrt((glo_workBenches[i].loc.x - glo_workBenches[j].loc.x) *
                                             (glo_workBenches[i].loc.x - glo_workBenches[j].loc.x)
                                             + (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y) *
                                               (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y));
                        dis_all.push_back(make_pair(dis_temp,table_temp));
                    }
                }
            }
            if(((glo_workBenches[i].sta_component & (1<<6))==0) && ( check_robotlink(i,2,6)))
            {
                for(int j=0;j<glo_count_workBench;j++)
                {
                    if(glo_workBenches[j].classID==6 && (( (glo_workBenches[j].sta_produce || (glo_workBenches[j].last<=30 &&glo_workBenches[j].last>0))) ) && check_robotlink(j,1,0))
                    {
                        vector<int> table_temp;
                        table_temp.push_back(j);
                        table_temp.push_back(i);
                        double dis_temp=sqrt((glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x) *
                                             (glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x)
                                             + (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y) *
                                               (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y))+
                                        sqrt((glo_workBenches[i].loc.x - glo_workBenches[j].loc.x) *
                                             (glo_workBenches[i].loc.x - glo_workBenches[j].loc.x)
                                             + (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y) *
                                               (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y));
                        dis_all.push_back(make_pair(dis_temp,table_temp));
                    }
                }
            }
        }
        if(glo_workBenches[i].classID==6 && ((glo_workBenches[i].loc.x < 26 && map_id ==2) || map_id !=2) )
        {
            if(glo_workBenches[i].sta_component==12) continue;
            if((glo_workBenches[i].sta_component &(1<<2))==0 && ( check_robotlink(i,2,2))) //缺2
            {
                for(int j=0;j<glo_count_workBench;j++)
                {
                    if(glo_workBenches[j].classID==2&& (glo_workBenches[j].sta_produce || (glo_workBenches[j].last<=30 &&glo_workBenches[j].last>0)) &&
                            check_linknum(j)<=2)
                    {
                        vector<int> table_temp;
                        table_temp.push_back(j);
                        table_temp.push_back(i);
                        double dis_temp=sqrt((glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x) *
                                             (glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x)
                                             + (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y) *
                                               (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y))+
                                        sqrt((glo_workBenches[i].loc.x - glo_workBenches[j].loc.x) *
                                         (glo_workBenches[i].loc.x - glo_workBenches[j].loc.x)
                                         + (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y) *
                                           (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y));
                        dis_all.push_back(make_pair(dis_temp,table_temp));

                    }
                }
            }
            if((glo_workBenches[i].sta_component &(1<<3))==0 && (check_robotlink(i,2,3))) //缺3
            {

                for(int j=0;j<glo_count_workBench;j++)
                {
                    if(glo_workBenches[j].classID==3&& (glo_workBenches[j].sta_produce || (glo_workBenches[j].last<=30 &&glo_workBenches[j].last>0)) && check_linknum(j)<=2)
                    {
                        vector<int> table_temp;
                        table_temp.push_back(j);
                        table_temp.push_back(i);
                        double dis_temp=sqrt((glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x) *
                                             (glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x)
                                             + (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y) *
                                               (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y))+
                                        sqrt((glo_workBenches[i].loc.x - glo_workBenches[j].loc.x) *
                                             (glo_workBenches[i].loc.x - glo_workBenches[j].loc.x)
                                             + (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y) *
                                               (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y));
                        dis_all.push_back(make_pair(dis_temp,table_temp));
                     }

                }
            }


        }
        if(glo_workBenches[i].classID==5  && ((glo_workBenches[i].loc.x < 26 && map_id ==2) || map_id !=2) )
        {
            if(glo_workBenches[i].sta_component==10) continue;
            if((glo_workBenches[i].sta_component & (1<<1))==0&& check_robotlink(i,2,1)) //缺1
            {
                for(int j=0;j<glo_count_workBench;j++)
                {
                    if(glo_workBenches[j].classID==1&& (glo_workBenches[j].sta_produce || (glo_workBenches[j].last<=30 &&glo_workBenches[j].last>0))&& check_linknum(j)<=2) {
                        vector<int> table_temp;
                        table_temp.push_back(j);
                        table_temp.push_back(i);
                        double dis_temp=sqrt((glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x) *
                                             (glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x)
                                             + (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y) *
                                               (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y))+
                                        sqrt((glo_workBenches[i].loc.x - glo_workBenches[j].loc.x) *
                                             (glo_workBenches[i].loc.x - glo_workBenches[j].loc.x)
                                             + (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y) *
                                               (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y));
                        dis_all.push_back(make_pair(dis_temp,table_temp));
                     }
                }
            }
            if((glo_workBenches[i].sta_component &(1<<3))==0&& (check_robotlink(i,2,3))) //缺3
            {
                for(int j=0;j<glo_count_workBench;j++)
                {
                    if(glo_workBenches[j].classID==3&& (glo_workBenches[j].sta_produce || (glo_workBenches[j].last<=30 &&glo_workBenches[j].last>0))&& check_linknum(j)<=2) {
                        vector<int> table_temp;
                        table_temp.push_back(j);
                        table_temp.push_back(i);
                        double dis_temp=sqrt((glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x) *
                                             (glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x)
                                             + (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y) *
                                               (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y))+
                                        sqrt((glo_workBenches[i].loc.x - glo_workBenches[j].loc.x) *
                                             (glo_workBenches[i].loc.x - glo_workBenches[j].loc.x)
                                             + (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y) *
                                               (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y));
                        dis_all.push_back(make_pair(dis_temp,table_temp));
                     }
                }
            }
        }
        if(glo_workBenches[i].classID==4)
        {
            if(glo_workBenches[i].sta_component==6) continue;
            if((glo_workBenches[i].sta_component &(1<<1))==0&& (check_robotlink(i,2,1))) //缺1
            {
                for(int j=0;j<glo_count_workBench;j++)
                {
                    if(glo_workBenches[j].classID==1 && (glo_workBenches[j].sta_produce || (glo_workBenches[j].last<=30 &&glo_workBenches[j].last>0))&& check_linknum(j)<=2) {
                        vector<int> table_temp;
                        table_temp.push_back(j);
                        table_temp.push_back(i);
                        double dis_temp=sqrt((glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x) *
                                             (glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x)
                                             + (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y) *
                                               (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y))+
                                        sqrt((glo_workBenches[i].loc.x - glo_workBenches[j].loc.x) *
                                             (glo_workBenches[i].loc.x - glo_workBenches[j].loc.x)
                                             + (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y) *
                                               (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y));
                        dis_all.push_back(make_pair(dis_temp,table_temp));
                    }
                }
            }
            if((glo_workBenches[i].sta_component &(1<<2))==0&& ( check_robotlink(i,2,2))) //缺2
            {
                for(int j=0;j<glo_count_workBench;j++)
                {
                    if(glo_workBenches[j].classID==2 && (glo_workBenches[j].sta_produce || (glo_workBenches[j].last<=30 &&glo_workBenches[j].last>0))&& check_linknum(j)<=2) {
                        vector<int> table_temp;
                        table_temp.push_back(j);
                        table_temp.push_back(i);
                        double dis_temp=sqrt((glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x) *
                                             (glo_robots[robot_id].loc.x - glo_workBenches[j].loc.x)
                                             + (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y) *
                                               (glo_robots[robot_id].loc.y - glo_workBenches[j].loc.y))+
                                        sqrt((glo_workBenches[i].loc.x - glo_workBenches[j].loc.x) *
                                             (glo_workBenches[i].loc.x - glo_workBenches[j].loc.x)
                                             + (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y) *
                                               (glo_workBenches[i].loc.y - glo_workBenches[j].loc.y));
                        dis_all.push_back(make_pair(dis_temp,table_temp));
                    }
                }
            }
        }
    }
    sort(dis_all.begin(),dis_all.end(), MyPaircmp1);
    if(dis_all.size()==0)//啥也没有
    {
        glo_robots[robot_id].no_chice= true;
        return no_choice;
    }
    glo_robots[robot_id].no_chice=false;
    return cost_fun(dis_all,robot_id);

}
int System::adiust_dynamic()
{
    if(map_id==1)
    {
        int num4=0;
        int num5=0;
        int num6=0;
        vector<int> num;
        for(int i=0;i<glo_count_workBench;i++)
        {
            if(glo_workBenches[i].classID==7)
            {
                if((glo_workBenches[i].sta_component &(1<<4))==0) num4++;
                if((glo_workBenches[i].sta_component &(1<<5))==0) num5++;
                if((glo_workBenches[i].sta_component &(1<<6))==0) num6++;
            }
            if(glo_workBenches[i].classID==4&&(glo_workBenches[i].sta_produce||glo_workBenches[i].last>0)) num4--;
            if(glo_workBenches[i].classID==5&&(glo_workBenches[i].sta_produce||glo_workBenches[i].last>0)) num5--;
            if(glo_workBenches[i].classID==6&&(glo_workBenches[i].sta_produce||glo_workBenches[i].last>0)) num6--;
        }
        for(int i=0;i<4;i++)
        {
            if(glo_workBenches[glo_robots[i].v_target_tables[1]].classID==4) num4--;
            if(glo_workBenches[glo_robots[i].v_target_tables[1]].classID==5) num5--;
            if(glo_workBenches[glo_robots[i].v_target_tables[1]].classID==6) num6--;
        }
        num.push_back(num4);
        num.push_back(num5);
        num.push_back(num6);
        sort(num.begin(),num.end());
        if(num[2]-num[1]>=1)
        {

            if(num[2]==num4) return 4;

            if(num[2]==num5) return 5;

            if(num[2]==num6) return 6;

        }
    }
    if(map_id==2)
    {
        int num4=0;
        int num5=0;
        int num6=0;
        vector<int> num;
        for(int i=0;i<glo_count_workBench;i++)
        {
            if(glo_workBenches[i].classID==7)
            {
                if((glo_workBenches[i].sta_component &(1<<4))==0) num4++;
                if((glo_workBenches[i].sta_component &(1<<5))==0) num5++;
                if((glo_workBenches[i].sta_component &(1<<6))==0) num6++;
            }
            if(glo_workBenches[i].classID==4&&(glo_workBenches[i].sta_produce||glo_workBenches[i].last>0)) num4--;
            if(glo_workBenches[i].classID==5&&(glo_workBenches[i].sta_produce||glo_workBenches[i].last>0)) num5--;
            if(glo_workBenches[i].classID==6&&(glo_workBenches[i].sta_produce||glo_workBenches[i].last>0)) num6--;

        }
        for(int i=0;i<4;i++)
        {
            if(glo_workBenches[glo_robots[i].v_target_tables[1]].classID==4) num4--;
            if(glo_workBenches[glo_robots[i].v_target_tables[1]].classID==5) num5--;
            if(glo_workBenches[glo_robots[i].v_target_tables[1]].classID==6) num6--;
        }
        num.push_back(num4);
        num.push_back(num5);
        num.push_back(num6);
        sort(num.begin(),num.end());
        // managerLog.addLog("woyaokan:"+to_string(num[0])+" , "+to_string(num[1])+" , "+to_string(num[2]));
        if(num[2]-num[1]>=1)
        {

            if(num[2]==num4) return 4;

            if(num[2]==num5) return 5;

            if(num[2]==num6) return 6;

        }
    }
    return 0;
}
vector<int> System::cost_fun(vector<pair<double,vector<int>>> dis_all, int robot_id)
{
    vector<pair<double,vector<int>>> dis_first;
    vector<pair<double,vector<int>>> dis_second;
    vector<pair<double,vector<int>>> lost_rate;
    vector<pair<double,vector<int>>> monney;
    vector<pair<double,vector<int>>> best_choice;
    for(int i=0;i<dis_all.size();i++)
    {
       double temp_dis_1=sqrt((glo_robots[robot_id].loc.x - glo_workBenches[dis_all[i].second[0]].loc.x) *
                        (glo_robots[robot_id].loc.x - glo_workBenches[dis_all[i].second[0]].loc.x)
                        + (glo_robots[robot_id].loc.y - glo_workBenches[dis_all[i].second[0]].loc.y) *
                          (glo_robots[robot_id].loc.y - glo_workBenches[dis_all[i].second[0]].loc.y));
       double temp_dis_2 = sqrt((glo_workBenches[dis_all[i].second[1]].loc.x - glo_workBenches[dis_all[i].second[0]].loc.x) *
                                (glo_workBenches[dis_all[i].second[1]].loc.x - glo_workBenches[dis_all[i].second[0]].loc.x)
                                + (glo_workBenches[dis_all[i].second[1]].loc.y - glo_workBenches[dis_all[i].second[0]].loc.y) *
                                  (glo_workBenches[dis_all[i].second[1]].loc.y - glo_workBenches[dis_all[i].second[0]].loc.y));
       dis_first.push_back(make_pair(temp_dis_1,dis_all[i].second));
       dis_second.push_back(make_pair(temp_dis_2,dis_all[i].second));
    }
    for(int i=0;i<dis_second.size();i++)
    {
        double lost_rate_temp;
        lost_rate_temp= cal_lost((dis_second[i].first/6+0.5)*50);
        lost_rate.push_back(make_pair(lost_rate_temp,dis_second[i].second));
    }
    for(int i=0;i<lost_rate.size();i++)
    {
        double monney_temp=0;
        if(glo_workBenches[lost_rate[i].second[0]].classID==1)
        {
            if(map_id==1)
            {
                monney_temp=6000*lost_rate[i].first-3000+200;
                if(glo_workBenches[lost_rate[i].second[1]].classID==4 && adiust_dynamic()==4)
                {

                    monney_temp=monney_temp+1500;
                }
                if(glo_workBenches[lost_rate[i].second[1]].classID==5 && adiust_dynamic()==5)
                {
                    monney_temp=monney_temp+1250;
                }
            }
            if(map_id==2)
            {
                monney_temp=6000*lost_rate[i].first-3000;
                if(glo_workBenches[lost_rate[i].second[1]].classID==4 && adiust_dynamic()==4)
                {

                    monney_temp=monney_temp+1200;
                }
                if(glo_workBenches[lost_rate[i].second[1]].classID==5 && adiust_dynamic()==5)
                {
                    monney_temp=monney_temp+1050;
                }
            }
            if(map_id==3)
            {
                monney_temp=6000*lost_rate[i].first-3000;
            }
            if(map_id==4)
            {
                monney_temp=6000*lost_rate[i].first-3000;
                if(glo_workBenches[lost_rate[i].second[1]].classID==4)
                {
                    monney_temp=monney_temp+3000;
                }
            }
            if(glo_workBenches[lost_rate[i].second[1]].classID==4 && (glo_workBenches[lost_rate[i].second[1]].sta_component==4 || check_willhave(lost_rate[i].second[1],2)))
            {
                if(map_id==1)
                {
                    monney_temp=monney_temp+850;
                }
                if(map_id==2)
                {
                    monney_temp=monney_temp+350;
                }
                if(map_id==3)
                {
                    monney_temp=monney_temp+350;
                }
                if(map_id==4)
                {
                    monney_temp=monney_temp+350;
                }
                if(glo_workBenches[lost_rate[i].second[1]].sta_produce)
                {
                    if(map_id==1)
                    {
                        monney_temp=monney_temp+850;
                    }
                    if(map_id==2)
                    {
                        monney_temp=monney_temp+350;
                    }
                    if(map_id==3)
                    {
                        monney_temp=monney_temp+350;
                    }
                    if(map_id==4)
                    {
                        monney_temp=monney_temp+3000;
                    }

                }
            }
            if(glo_workBenches[lost_rate[i].second[1]].classID==5 && (glo_workBenches[lost_rate[i].second[1]].sta_component==8|| check_willhave(lost_rate[i].second[1],3)))
            {
                if(map_id==1)
                {
                    monney_temp=monney_temp+900;
                }
                if(map_id==2)
                {
                    monney_temp=monney_temp+400;
                }
                if(map_id==3)
                {
                    monney_temp=monney_temp+400;
                }
                if(map_id==4)
                {
                    monney_temp=monney_temp+800;
                }

                if(glo_workBenches[lost_rate[i].second[1]].sta_produce)
                {
                    if(map_id==1)
                    {
                        monney_temp=monney_temp+900;
                    }
                    if(map_id==2)
                    {
                        monney_temp=monney_temp+400;
                    }
                    if(map_id==3)
                    {
                        monney_temp=monney_temp+400;
                    }
                    if(map_id==4)
                    {
                        monney_temp=monney_temp+400;
                    }

                }
            }
            monney.push_back(make_pair(monney_temp,lost_rate[i].second));
        }
        if(glo_workBenches[lost_rate[i].second[0]].classID==2)
        {
            if(map_id==1)
            {
                monney_temp=7600*lost_rate[i].first-4400;
                if(glo_workBenches[lost_rate[i].second[1]].classID==4 && adiust_dynamic()==4)
                {
                    monney_temp=monney_temp+1500;
                }

                if(glo_workBenches[lost_rate[i].second[1]].classID==6 && adiust_dynamic()==6)
                {
                    monney_temp=monney_temp+1000;
                }
            }
            if(map_id==2)
            {
                monney_temp=7600*lost_rate[i].first-4400;
                if(glo_workBenches[lost_rate[i].second[1]].classID==4 && adiust_dynamic()==4)
                {

                    monney_temp=monney_temp+1200;
                }
//                if(glo_workBenches[lost_rate[i].second[1]].classID==6)
//                {
//                    monney_temp=monney_temp+600;
//                }
                if(glo_workBenches[lost_rate[i].second[1]].classID==6 && adiust_dynamic()==6)
                {
                    monney_temp=monney_temp+1550;
                }
            }
            if(map_id==3)
            {
                monney_temp=7600*lost_rate[i].first-4400;
                if(glo_workBenches[lost_rate[i].second[1]].classID==5)
                {
                    monney_temp=monney_temp+600;
                }
            }
            if(map_id==4)
            {
                monney_temp=7600*lost_rate[i].first-4400;
                if(glo_workBenches[lost_rate[i].second[1]].classID==4)
                {
                    monney_temp=monney_temp+2000;
                }
            }

            if(glo_workBenches[lost_rate[i].second[1]].classID==4 && (glo_workBenches[lost_rate[i].second[1]].sta_component==2|| check_willhave(lost_rate[i].second[1],1)))
            {
                if(map_id==1)
                {
                    monney_temp=monney_temp+850;
                }
                if(map_id==2)
                {
                    monney_temp=monney_temp+350;
                }
                if(map_id==3)
                {
                    monney_temp=monney_temp+350;
                }
                if(map_id==4)
                {
                    monney_temp=monney_temp+350;
                }

                if(glo_workBenches[lost_rate[i].second[1]].sta_produce)
                {
                    if(map_id==1)
                    {
                        monney_temp=monney_temp+850;
                    }
                    if(map_id==2)
                    {
                        monney_temp=monney_temp+350;
                    }
                    if(map_id==3)
                    {
                        monney_temp=monney_temp+350;
                    }
                    if(map_id==4)
                    {
                        monney_temp=monney_temp+2500;
                    }

                }
            }
            if(glo_workBenches[lost_rate[i].second[1]].classID==6 && (glo_workBenches[lost_rate[i].second[1]].sta_component==8|| check_willhave(lost_rate[i].second[1],3)))
            {
                if(map_id==1)
                {
                    monney_temp=monney_temp+1000;
                }
                if(map_id==2)
                {
                    monney_temp=monney_temp+500;        //正在修改值
                }
                if(map_id==3)
                {
                    monney_temp=monney_temp+1000;
                }
                if(map_id==4)
                {
                    monney_temp=monney_temp+500;
                }
                if(glo_workBenches[lost_rate[i].second[1]].sta_produce)
                {
                    if(map_id==1)
                    {
                        monney_temp=monney_temp+950;
                    }
                    if(map_id==2)
                    {
                        monney_temp=monney_temp+2500;       //正在修改值
                    }
                    if(map_id==3)
                    {
                        monney_temp=monney_temp+1000;
                    }
                    if(map_id==4)
                    {
                        monney_temp=monney_temp+450;
                    }

                }
            }
            monney.push_back(make_pair(monney_temp,lost_rate[i].second));
        }
        if(glo_workBenches[lost_rate[i].second[0]].classID==3)
        {
            if(map_id==1)
            {
                monney_temp=9200*lost_rate[i].first-5800;
                if(glo_workBenches[lost_rate[i].second[1]].classID==5 && adiust_dynamic()==5)
                {
                    monney_temp=monney_temp+1550;
                }
                if(glo_workBenches[lost_rate[i].second[1]].classID==6 && adiust_dynamic()==6)
                {
                    monney_temp=monney_temp+1000;
                }
            }
            if(map_id==2)
            {
                monney_temp=9200*lost_rate[i].first-5800;
                if(glo_workBenches[lost_rate[i].second[1]].classID==5 && adiust_dynamic()==5)
                {

                    monney_temp=monney_temp+1200;
                }
//                if(glo_workBenches[lost_rate[i].second[1]].classID==6)
//                {
//                    monney_temp=monney_temp+600;
//                }
                if(glo_workBenches[lost_rate[i].second[1]].classID==6 && adiust_dynamic()==6)
                {
                    monney_temp=monney_temp+1550;
                }
            }
            if(map_id==3)
            {
                monney_temp=9200*lost_rate[i].first-5800;
                if(glo_workBenches[lost_rate[i].second[1]].classID==5)
                {
                    monney_temp=monney_temp+600;
                }
            }
            if(map_id==4)
            {
                monney_temp=9200*lost_rate[i].first-5800-1000;
            }
            if(glo_workBenches[lost_rate[i].second[1]].classID==5 && (glo_workBenches[lost_rate[i].second[1]].sta_component==2|| check_willhave(lost_rate[i].second[1],1)))
            {
                if(map_id==1)
                {
                    monney_temp=monney_temp+850;
                }
                if(map_id==2)
                {
                    monney_temp=monney_temp+400;
                }
                if(map_id==3)
                {
                    monney_temp=monney_temp+400;
                }
                if(map_id==4)
                {
                    monney_temp=monney_temp+400;
                }

                if(glo_workBenches[lost_rate[i].second[1]].sta_produce)
                {
                    if(map_id==1)
                    {
                        monney_temp=monney_temp+850;
                    }
                    if(map_id==2)
                    {
                        monney_temp=monney_temp+400;
                    }
                    if(map_id==3)
                    {
                        monney_temp=monney_temp+400;
                    }
                    if(map_id==4)
                    {
                        monney_temp=monney_temp+400;
                    }
                }
            }
            if(glo_workBenches[lost_rate[i].second[1]].classID==6 && (glo_workBenches[lost_rate[i].second[1]].sta_component==4|| check_willhave(lost_rate[i].second[1],2)))
            {
                if(map_id==1)
                {
                    monney_temp=monney_temp+1000;
                }
                if(map_id==2)
                {
                    monney_temp=monney_temp+500;       //正在修改值
                }
                if(map_id==3)
                {
                    monney_temp=monney_temp+1000;
                }
                if(map_id==4)
                {
                    monney_temp=monney_temp+500;
                }

                if(glo_workBenches[lost_rate[i].second[1]].sta_produce)
                {
                    if(map_id==1)
                    {
                        monney_temp=monney_temp+1000;
                    }
                    if(map_id==2)
                    {
                        monney_temp=monney_temp+2500;       //正在修改值
                    }
                    if(map_id==3)
                    {
                        monney_temp=monney_temp+1000;
                    }
                    if(map_id==4)
                    {
                        monney_temp=monney_temp+450;
                    }

                }
            }
            monney.push_back(make_pair(monney_temp,lost_rate[i].second));
        }
        if(glo_workBenches[lost_rate[i].second[0]].classID==4)
        {
            monney_temp=22500*lost_rate[i].first-15400;
            if(map_id==1)
            {
                monney_temp=monney_temp+(glo_workBenches[lost_rate[i].second[1]].ID-10)*5000;
            }
            if(map_id==2)
            {
                monney_temp=monney_temp;
            }
            if(map_id==3)
            {
                monney_temp=monney_temp;
            }
            if(map_id==4)
            {
                monney_temp=monney_temp;
            }
            if(glo_workBenches[lost_rate[i].second[1]].classID==7 && (glo_workBenches[lost_rate[i].second[1]].sta_component==96 || ((glo_workBenches[lost_rate[i].second[1]].sta_component & (1<<5)) &&
                    check_willhave(lost_rate[i].second[1],6)) ||  ((glo_workBenches[lost_rate[i].second[1]].sta_component & (1<<6)) && check_willhave(lost_rate[i].second[1],5))
                    || (check_willhave(lost_rate[i].second[1],5) && check_willhave(lost_rate[i].second[1],6))))
            {
                if(map_id==1)
                {
                    monney_temp=monney_temp+1800+(glo_workBenches[lost_rate[i].second[1]].ID-10)*200;
                }
                if(map_id==2)
                {
                    monney_temp=monney_temp+1200;
                }
                if(map_id==3)
                {
                    monney_temp=monney_temp+1200;
                }
                if(map_id==4)
                {
                    monney_temp=monney_temp-1000;
                }

                if(glo_workBenches[lost_rate[i].second[1]].sta_produce)
                {
                    if(map_id==1)
                    {
                        monney_temp=monney_temp+1200;
                    }
                    if(map_id==2)
                    {
                        monney_temp=monney_temp+1200;
                    }
                    if(map_id==3)
                    {
                        monney_temp=monney_temp+1200;
                    }
                    if(map_id==4)
                    {
                        monney_temp=monney_temp-1000;
                    }

                }
            }
            monney.push_back(make_pair(monney_temp,lost_rate[i].second));
        }
        if(glo_workBenches[lost_rate[i].second[0]].classID==5)
        {
            monney_temp=25000*lost_rate[i].first-17200;
            if(map_id==1)
            {
                monney_temp=monney_temp+(glo_workBenches[lost_rate[i].second[1]].ID-10)*1000;
            }
            if(map_id==2)
            {
                monney_temp=monney_temp;
            }
            if(map_id==3)
            {
                monney_temp=monney_temp;
            }
            if(map_id==4)
            {
                monney_temp=monney_temp;
            }
            if(glo_workBenches[lost_rate[i].second[1]].classID==7 && (glo_workBenches[lost_rate[i].second[1]].sta_component==80|| ((glo_workBenches[lost_rate[i].second[1]].sta_component & (1<<6)) &&
              check_willhave(lost_rate[i].second[1],4)) ||  ((glo_workBenches[lost_rate[i].second[1]].sta_component & (1<<4)) && check_willhave(lost_rate[i].second[1],6))
               || (check_willhave(lost_rate[i].second[1],4) && check_willhave(lost_rate[i].second[1],6))))
            {
                if(map_id==1)
                {
                    monney_temp=monney_temp+2300+(glo_workBenches[lost_rate[i].second[1]].ID-10)*200;
                }
                if(map_id==2)
                {
                    monney_temp=monney_temp+1300;
                }
                if(map_id==3)
                {
                    monney_temp=monney_temp+1300;
                }
                if(map_id==4)
                {
                    monney_temp=monney_temp+1300;
                }

                if(glo_workBenches[lost_rate[i].second[1]].sta_produce)
                {
                    if(map_id==1)
                    {
                        monney_temp=monney_temp+1200;
                    }
                    if(map_id==2)
                    {
                        monney_temp=monney_temp+1200;
                    }
                    if(map_id==3)
                    {
                        monney_temp=monney_temp+1200;
                    }
                    if(map_id==4)
                    {
                        monney_temp=monney_temp+1200;
                    }

                }
            }
            monney.push_back(make_pair(monney_temp,lost_rate[i].second));
        }
        if(glo_workBenches[lost_rate[i].second[0]].classID==6)
        {
            monney_temp=27500*lost_rate[i].first-19200;
            if(map_id==1)
            {
                monney_temp=monney_temp+(glo_workBenches[lost_rate[i].second[1]].ID-10)*1000;
            }
            if(map_id==2)
            {
                monney_temp=monney_temp;
            }
            if(map_id==3)
            {
                monney_temp=monney_temp+800;
            }
            if(map_id==4)
            {
                monney_temp=monney_temp;
            }
            if(glo_workBenches[lost_rate[i].second[1]].classID==7 && (glo_workBenches[lost_rate[i].second[1]].sta_component==48 || ((glo_workBenches[lost_rate[i].second[1]].sta_component & (1<<5)) &&
           check_willhave(lost_rate[i].second[1],4)) ||  ((glo_workBenches[lost_rate[i].second[1]].sta_component & (1<<4)) && check_willhave(lost_rate[i].second[1],5))
               || (check_willhave(lost_rate[i].second[1],5) && check_willhave(lost_rate[i].second[1],4))))
            {
                if(map_id==1)
                {
                    monney_temp=monney_temp+2500+(glo_workBenches[lost_rate[i].second[1]].ID-10)*200;
                }
                if(map_id==2)
                {
                    monney_temp=monney_temp-1000;        //正在修改值
                }
                if(map_id==3)
                {
                    monney_temp=monney_temp+1500;
                }
                if(map_id==4)
                {
                    monney_temp=monney_temp+1500;
                }

                if(glo_workBenches[lost_rate[i].second[1]].sta_produce)
                {
                    if(map_id==1)
                    {
                        monney_temp=monney_temp+1200;
                    }
                    if(map_id==2)
                    {
                        monney_temp=monney_temp-1000;        //正在修改值
                    }
                    if(map_id==3)
                    {
                        monney_temp=monney_temp+1200;
                    }
                    if(map_id==4)
                    {
                        monney_temp=monney_temp+1200;
                    }

                }
            }
            monney.push_back(make_pair(monney_temp,lost_rate[i].second));
        }
        if(glo_workBenches[lost_rate[i].second[0]].classID==7)
        {
            if(map_id==1)
            {
                monney_temp=105000*lost_rate[i].first-76000;
            }
            if(map_id==2)
            {
                monney_temp=105000*lost_rate[i].first-76000;
            }
            if(map_id==3)
            {
                monney_temp=105000*lost_rate[i].first-76000;
            }
            if(map_id==4)
            {
                monney_temp=105000*lost_rate[i].first-76000-12000;
            }

            monney.push_back(make_pair(monney_temp,lost_rate[i].second));
        }
    }
   for(int i=0;i<monney.size();i++)
    {
        double best_temp;
        best_temp=monney[i].first/((dis_first[i].first+dis_second[i].first)/6+1);
        best_choice.push_back(make_pair(best_temp,monney[i].second));
    }
    sort(best_choice.begin(),best_choice.end(), MyPaircmp2);
    return best_choice[0].second;

}
double System::cal_lost(int fps)
{
    double maxX=9000;
    double mini_rate=0.8;
    double ans;
    if(fps<maxX)
    {
        ans=(1- sqrt((1-(1-fps/maxX)*(1-fps/maxX))))*(1-mini_rate)+mini_rate;
    }
    else
    {
        ans=mini_rate;
    }
    return ans;
}
void System::robot_actionv2()
{
    vector<int> temp;
    temp.push_back(5);
    temp.push_back(8);
    for(int i=0;i<4;i++)
    {
        vector<int> tar_temp;
        if(!glo_robots[i].busy )
        {
            tar_temp=scan_mostneed(i);
            if(glo_robots[i].no_chice)
            {

            }
            else
            {
                glo_robots[i].v_target_tables=tar_temp;
                glo_workBenches[glo_robots[i].v_target_tables[0]].robot[i]=1;
                glo_workBenches[glo_robots[i].v_target_tables[1]].robot[i]=2;
                glo_robots[i].arrived_1= false;
                glo_robots[i].arrived_2= false;
                glo_robots[i].busy= true;
            }

        }
    }
    for(int i=0;i<4;i++)
    {
        if(!glo_robots[i].arrived_1 && !glo_robots[i].arrived_2 && glo_robots[i].busy)
        {
            //set_Pointv4(i,glo_workBenches[5].loc.x,glo_workBenches[5].loc.y);
            glo_robots[i].target_table=glo_robots[i].v_target_tables[0];
            set_Pointv4(i,glo_workBenches[glo_robots[i].target_table].loc.x,glo_workBenches[glo_robots[i].target_table].loc.y);
            if(glo_robots[i].arrived && glo_frameID<=stop_time)
            {
                control(glo_robots[i].ID, BUY_ACTION);
                if(glo_robots[i].objectID!=0)
                {
                    glo_robots[i].arrived_1= true;
                    glo_workBenches[glo_robots[i].target_table].robot[i]=0;
                }

            }
        }
        if(glo_robots[i].arrived_1 && !glo_robots[i].arrived_2 && glo_robots[i].busy)
        {
            glo_robots[i].target_table=glo_robots[i].v_target_tables[1];
            set_Pointv4(i,glo_workBenches[glo_robots[i].v_target_tables[1]].loc.x,glo_workBenches[glo_robots[i].v_target_tables[1]].loc.y);
            if(glo_robots[i].arrived)
            {
                control(glo_robots[i].ID, SELL_ACTION);

                if(glo_robots[i].objectID==0)
                {
                    glo_robots[i].arrived_2= true;
                    glo_workBenches[glo_robots[i].target_table].robot[i]=0;
                    glo_robots[i].busy= false;
                }
            }
        }
    }

}
float System::avoid_obs(int8_t robotID)
{
    vector<pair<float, int>> each_rob_dis3;
    vector<pair<float, int>> each_rob_dis2;
    vector<pair<float, int>> each_rob_dis1;
    vector<pair<float, int>> each_rob_dis0;

    vector<pair<float, float>> each_rob_direct3;
    vector<pair<float, float>> each_rob_direct2;
    vector<pair<float, float>> each_rob_direct1;
    vector<pair<float, float>> each_rob_direct0;

    int distForR2R = 2;
    int distRadius = 3.5;
    int distLine = 3;
    float ang_big = M_PI/3;
    int robSpeed =  sqrt(glo_robots[robotID].vel_line.x * glo_robots[robotID].vel_line.x + glo_robots[robotID].vel_line.y * glo_robots[robotID].vel_line.y);
    if(map_id == 3)
    {
        if(glo_robots[robotID].loc.x < 37)
        {
            distForR2R = 4;
            distRadius = 2;
            distLine = 1.5;
            ang_big = M_PI/3;
        }
        else
        {
            distForR2R = 2;
            distRadius = 2;
            distLine = 1.5;
            ang_big = M_PI/3;
        }
    }
    else
    {
        if(robSpeed >= 4)
        {
            distForR2R = 4;
            distRadius = 2.5;
            distLine = 2;
            ang_big = M_PI/3;
        }
        else
        {
            distForR2R = 2;
            distRadius = 2;
            distLine = 1.5;
            ang_big = M_PI/3;
        }
    }

    float ang_range = M_PI/2;
    float w = -1;
    for(int cnt_rob = 0; cnt_rob < 4; cnt_rob++)
    {
        if(cnt_rob == 3)
        {
            for(int i = 0; i < 4; i++)
            {
                if(i != cnt_rob)
                {
                    each_rob_dis3.push_back(make_pair(getDistR2R(glo_robots[cnt_rob], glo_robots[i]), i));

                    float body_rob_x =  (glo_robots[i].loc.x-glo_robots[cnt_rob].loc.x)*cos(glo_robots[cnt_rob].direct)+(glo_robots[i].loc.y-glo_robots[cnt_rob].loc.y)*sin(glo_robots[cnt_rob].direct);
                    float body_rob_y = -(glo_robots[i].loc.x-glo_robots[cnt_rob].loc.x)*sin(glo_robots[cnt_rob].direct)+(glo_robots[i].loc.y-glo_robots[cnt_rob].loc.y)*cos(glo_robots[cnt_rob].direct);
                    float r_direct1 = atan2(body_rob_y,body_rob_x);
                    each_rob_direct3.push_back(make_pair(getDistR2R(glo_robots[cnt_rob], glo_robots[i]), r_direct1));
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
        }
        else if(cnt_rob == 2)
        {
            for(int i = 0; i < 4; i++)
            {
                if(i != cnt_rob)
                {
                    each_rob_dis2.push_back(make_pair(getDistR2R(glo_robots[cnt_rob], glo_robots[i]), i));

                    float body_rob_x =  (glo_robots[i].loc.x-glo_robots[cnt_rob].loc.x)*cos(glo_robots[cnt_rob].direct)+(glo_robots[i].loc.y-glo_robots[cnt_rob].loc.y)*sin(glo_robots[cnt_rob].direct);
                    float body_rob_y = -(glo_robots[i].loc.x-glo_robots[cnt_rob].loc.x)*sin(glo_robots[cnt_rob].direct)+(glo_robots[i].loc.y-glo_robots[cnt_rob].loc.y)*cos(glo_robots[cnt_rob].direct);
                    float r_direct1 = atan2(body_rob_y,body_rob_x);
                    each_rob_direct2.push_back(make_pair(getDistR2R(glo_robots[cnt_rob], glo_robots[i]), r_direct1));
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

        }
        else if(cnt_rob == 1)
        {
            for(int i = 0; i < 4; i++)
            {
                if(i != cnt_rob)
                {
                    each_rob_dis1.push_back(make_pair(getDistR2R(glo_robots[cnt_rob], glo_robots[i]), i));

                    float body_rob_x =  (glo_robots[i].loc.x-glo_robots[cnt_rob].loc.x)*cos(glo_robots[cnt_rob].direct)+(glo_robots[i].loc.y-glo_robots[cnt_rob].loc.y)*sin(glo_robots[cnt_rob].direct);
                    float body_rob_y = -(glo_robots[i].loc.x-glo_robots[cnt_rob].loc.x)*sin(glo_robots[cnt_rob].direct)+(glo_robots[i].loc.y-glo_robots[cnt_rob].loc.y)*cos(glo_robots[cnt_rob].direct);
                    float r_direct1 = atan2(body_rob_y,body_rob_x);
                    each_rob_direct1.push_back(make_pair(getDistR2R(glo_robots[cnt_rob], glo_robots[i]), r_direct1));
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

        }
        else if(cnt_rob == 0)
        {
            for(int i = 0; i < 4; i++)
            {
                if(i != cnt_rob)
                {
                    each_rob_dis0.push_back(make_pair(getDistR2R(glo_robots[cnt_rob], glo_robots[i]), i));

                    float body_rob_x =  (glo_robots[i].loc.x-glo_robots[cnt_rob].loc.x)*cos(glo_robots[cnt_rob].direct)+(glo_robots[i].loc.y-glo_robots[cnt_rob].loc.y)*sin(glo_robots[cnt_rob].direct);
                    float body_rob_y = -(glo_robots[i].loc.x-glo_robots[cnt_rob].loc.x)*sin(glo_robots[cnt_rob].direct)+(glo_robots[i].loc.y-glo_robots[cnt_rob].loc.y)*cos(glo_robots[cnt_rob].direct);
                    float r_direct1 = atan2(body_rob_y,body_rob_x);
                    each_rob_direct0.push_back(make_pair(getDistR2R(glo_robots[cnt_rob], glo_robots[i]), r_direct1));
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

        }
    }
    if(robotID == 0)
    {
        for(int i=0; i<each_rob_dis0.size(); i++)
        {
            if(each_rob_direct0[i].first <= distForR2R && (each_rob_direct0[i].second >= -ang_range && each_rob_direct0[i].second <= ang_range))
            {
                if(each_rob_direct0[i].first >= distRadius)
                {
                    if(each_rob_direct0[i].second <= asin(distLine/each_rob_direct0[i].first) && each_rob_direct0[i].second >= -asin(distLine/each_rob_direct0[i].first))
                    {
                        if(glo_robots[robotID].objectID == 0 && glo_robots[each_rob_dis0[i].second].objectID != 0)
                        {
                            if(each_rob_direct0[i].second >= 0 && each_rob_direct0[i].second <= ang_range)
                            {
                                w = -ang_big;
                            }
                            else
                            {
                                w = ang_big;
                            }
                        }
                        else if(glo_robots[robotID].objectID != 0 && glo_robots[each_rob_dis0[i].second].objectID == 0)
                        {

                        }
                        else if(robotID < each_rob_dis0[i].second)
                        {
                            if(each_rob_direct0[i].second >= 0 && each_rob_direct0[i].second <= ang_range)
                            {
                                w = -ang_big;
                            }
                            else
                            {
                                w = ang_big;
                            }
                        }

                        w = 10*w;
                        break;
                    }
                }
                else if(each_rob_direct0[i].first < distRadius)
                {
                    if(glo_robots[robotID].objectID == 0 && glo_robots[each_rob_dis0[i].second].objectID != 0)
                    {
                        if(each_rob_direct0[i].second >= 0 && each_rob_direct0[i].second <= ang_range)
                        {
                            w = -ang_big;
                        }
                        else
                        {
                            w = ang_big;
                        }
                    }
                    else if(glo_robots[robotID].objectID != 0 && glo_robots[each_rob_dis0[i].second].objectID == 0)
                    {

                    }
                    else if(robotID < each_rob_dis0[i].second)
                    {
                        if(each_rob_direct0[i].second >= 0 && each_rob_direct0[i].second <= ang_range)
                        {
                            w = -ang_big;
                        }
                        else
                        {
                            w = ang_big;
                        }
                    }

                    w = 10*w;
                    break;
                }
            }
        }
    }
    else if(robotID == 1)
    {
        for(int i=0; i<each_rob_dis1.size(); i++)
        {
            if(each_rob_direct1[i].first <= distForR2R && (each_rob_direct1[i].second >= -ang_range && each_rob_direct1[i].second <= ang_range))
            {
                if(each_rob_direct1[i].first >= distRadius)
                {
                    if(each_rob_direct1[i].second <= asin(distLine/each_rob_direct1[i].first) && each_rob_direct1[i].second >= -asin(distLine/each_rob_direct1[i].first))
                    {
                        if(glo_robots[robotID].objectID == 0 && glo_robots[each_rob_dis1[i].second].objectID != 0)
                        {
                            if(each_rob_direct1[i].second >= 0 && each_rob_direct1[i].second <= ang_range)
                            {
                                w = -ang_big;
                            }
                            else
                            {
                                w = ang_big;
                            }
                        }
                        else if(glo_robots[robotID].objectID != 0 && glo_robots[each_rob_dis1[i].second].objectID == 0)
                        {

                        }
                        else if(robotID < each_rob_dis1[i].second)
                        {
                            if(each_rob_direct1[i].second >= 0 && each_rob_direct1[i].second <= ang_range)
                            {
                                w = -ang_big;
                            }
                            else
                            {
                                w = ang_big;
                            }
                        }

                        w = 10*w;
                        break;
                    }
                }
                else if(each_rob_direct1[i].first < distRadius)
                {
                    if(glo_robots[robotID].objectID == 0 && glo_robots[each_rob_dis1[i].second].objectID != 0)
                    {
                        if(each_rob_direct1[i].second >= 0 && each_rob_direct1[i].second <= ang_range)
                        {
                            w = -ang_big;
                        }
                        else
                        {
                            w = ang_big;
                        }
                    }
                    else if(glo_robots[robotID].objectID != 0 && glo_robots[each_rob_dis1[i].second].objectID == 0)
                    {

                    }
                    else if(robotID < each_rob_dis1[i].second)
                    {
                        if(each_rob_direct1[i].second >= 0 && each_rob_direct1[i].second <= ang_range)
                        {
                            w = -ang_big;
                        }
                        else
                        {
                            w = ang_big;
                        }
                    }

                    w = 10*w;
                    break;
                }
            }
        }
    }
    else if(robotID == 2)
    {
        for(int i=0; i<each_rob_dis2.size(); i++)
        {
            if(each_rob_direct2[i].first <= distForR2R && (each_rob_direct2[i].second >= -ang_range && each_rob_direct2[i].second <= ang_range))
            {
                if(each_rob_direct2[i].first >= distRadius)
                {
                    if(each_rob_direct2[i].second <= asin(distLine/each_rob_direct2[i].first) && each_rob_direct2[i].second >= -asin(distLine/each_rob_direct2[i].first))
                    {
                        if(glo_robots[robotID].objectID == 0 && glo_robots[each_rob_dis2[i].second].objectID != 0)
                        {
                            if(each_rob_direct2[i].second >= 0 && each_rob_direct2[i].second <= ang_range)
                            {
                                w = -ang_big;
                            }
                            else
                            {
                                w = ang_big;
                            }
                        }
                        else if(glo_robots[robotID].objectID != 0 && glo_robots[each_rob_dis2[i].second].objectID == 0)
                        {

                        }
                        else if(robotID < each_rob_dis2[i].second)
                        {
                            if(each_rob_direct2[i].second >= 0 && each_rob_direct2[i].second <= ang_range)
                            {
                                w = -ang_big;
                            }
                            else
                            {
                                w = ang_big;
                            }
                        }

                        w = 10*w;
                        break;
                    }
                }
                else if(each_rob_direct2[i].first < distRadius)
                {
                    if(glo_robots[robotID].objectID == 0 && glo_robots[each_rob_dis2[i].second].objectID != 0)
                    {
                        if(each_rob_direct2[i].second >= 0 && each_rob_direct2[i].second <= ang_range)
                        {
                            w = -ang_big;
                        }
                        else
                        {
                            w = ang_big;
                        }
                    }
                    else if(glo_robots[robotID].objectID != 0 && glo_robots[each_rob_dis2[i].second].objectID == 0)
                    {

                    }
                    else if(robotID < each_rob_dis2[i].second)
                    {
                        if(each_rob_direct2[i].second >= 0 && each_rob_direct2[i].second <= ang_range)
                        {
                            w = -ang_big;
                        }
                        else
                        {
                            w = ang_big;
                        }
                    }

                    w = 10*w;
                    break;
                }
            }
        }
    }
    else if(robotID == 3)
    {
        for(int i=0; i<each_rob_dis3.size(); i++)
        {
            if(each_rob_direct3[i].first <= distForR2R && (each_rob_direct3[i].second >= -ang_range && each_rob_direct3[i].second <= ang_range))
            {
                if(each_rob_direct3[i].first >= distRadius)
                {
                    if(each_rob_direct3[i].second <= asin(distLine/each_rob_direct3[i].first) && each_rob_direct3[i].second >= -asin(distLine/each_rob_direct3[i].first))
                    {
                        if(glo_robots[robotID].objectID == 0 && glo_robots[each_rob_dis3[i].second].objectID != 0)
                        {
                            if(each_rob_direct3[i].second >= 0 && each_rob_direct3[i].second <= ang_range)
                            {
                                w = -ang_big;
                            }
                            else
                            {
                                w = ang_big;
                            }
                        }
                        else if(glo_robots[robotID].objectID != 0 && glo_robots[each_rob_dis3[i].second].objectID == 0)
                        {

                        }
                        else if(robotID < each_rob_dis3[i].second)
                        {
                            if(each_rob_direct3[i].second >= 0 && each_rob_direct3[i].second <= ang_range)
                            {
                                w = -ang_big;
                            }
                            else
                            {
                                w = ang_big;
                            }
                        }

                        w = 10*w;
                        break;
                    }
                }
                else if(each_rob_direct3[i].first < distRadius)
                {
                    if(glo_robots[robotID].objectID == 0 && glo_robots[each_rob_dis3[i].second].objectID != 0)
                    {
                        if(each_rob_direct3[i].second >= 0 && each_rob_direct3[i].second <= ang_range)
                        {
                            w = -ang_big;
                        }
                        else
                        {
                            w = ang_big;
                        }
                    }
                    else if(glo_robots[robotID].objectID != 0 && glo_robots[each_rob_dis3[i].second].objectID == 0)
                    {

                    }
                    else if(robotID < each_rob_dis3[i].second)
                    {
                        if(each_rob_direct3[i].second >= 0 && each_rob_direct3[i].second <= ang_range)
                        {
                            w = -ang_big;
                        }
                        else
                        {
                            w = ang_big;
                        }
                    }

                    w = 10*w;
                    break;
                }
            }
        }
    }
    return w;
}

bool System::set_Pointv4(uint8_t robotID,double ref_x,double ref_y){
    static float last_refx=ref_x;
    static float last_refy=ref_y;


    if (last_refx != ref_x || last_refy != ref_y){
        glo_robots[robotID].arrived= false;
    }
    float r_direct=0;
    float body_refx=(ref_x-glo_robots[robotID].loc.x)*cos(glo_robots[robotID].direct)+(ref_y-glo_robots[robotID].loc.y)*sin(glo_robots[robotID].direct);
    float body_refy = -(ref_x-glo_robots[robotID].loc.x)*sin(glo_robots[robotID].direct)+(ref_y-glo_robots[robotID].loc.y)*cos(glo_robots[robotID].direct);
//    if (abs(body_refy)<0.0001){//防止奇异跳变
//        r_direct=0;
//    }
//    else
//    {
    r_direct=atan2(body_refy,body_refx);
    //}
    float v=6.0f;
    float distance = sqrt((ref_y-glo_robots[robotID].loc.y)*(ref_y-glo_robots[robotID].loc.y)+(ref_x-glo_robots[robotID].loc.x)*(ref_x-glo_robots[robotID].loc.x));

    if(distance<=1) v=1;
    else v = 6;
    if (v>=6) v=6;
    if (v<=-2) v=-2;
    float w=r_direct;

    if(abs(w)>=M_PI_4) v=2;

    int r=0;
    int obj=glo_robots->objectID-'0';
    if(obj>0) r=0.53;
    else r=0.45;

    w = 10*w;
    if (w>=M_PI) w=M_PI;
    if (w<-M_PI) w=-M_PI;
    /*
    if(abs(r_direct)>=M_PI_4) v=-1;
    if(distance<=6 && abs(r_direct)>M_PI*2) v=-2;
    */

    if(map_id==3)
    {
        float ang = avoid_obs(robotID);
        if(ang != -1 && ang != -10)
            w = ang;
    }
    if(map_id==2)
    {
        float ang = avoid_obs2(robotID);
        if(ang != -1 && ang != -10)
            w = ang;
//        if(w!=0 && distance<0.5){
//            v=1.5;
//        }
    }
    if(map_id==1)
    {
        float ang = avoid_obs2(robotID);
        if(ang != -1 && ang != -10)
            w = ang;
    }
    if(map_id==4)
    {
        float ang = avoid_obs2(robotID);
        if(ang != -1 && ang != -10)
            w = ang;
    }

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
//    if(glo_robots[robotID].loc.x>=50-r || glo_robots[robotID].loc.x<=r ||
//       glo_robots[robotID].loc.y>=50-r || glo_robots[robotID].loc.y<=r)
//        v=-2.0f;
    //已经到达
    if (glo_robots[robotID].arrived)
    {
        v=0.0f;
    }
    last_refx=ref_x;
    last_refy=ref_y;
    control(robotID,FORWARD_ACTION,v);
    control(robotID,ROTATE_ACTION,w);
    if (glo_robots[robotID].workBenchID==glo_robots[robotID].target_table){
        glo_robots[robotID].arrived= true;
    }
    else {
        glo_robots[robotID].arrived=false;
    }
    return glo_robots[robotID].arrived;



}

float System::avoid_obs2(int robotID)
{
    vector<pair<float, int>> each_rob_dis3;
    vector<pair<float, int>> each_rob_dis2;
    vector<pair<float, int>> each_rob_dis1;
    vector<pair<float, int>> each_rob_dis0;

    vector<pair<float, float>> each_rob_direct3;
    vector<pair<float, float>> each_rob_direct2;
    vector<pair<float, float>> each_rob_direct1;
    vector<pair<float, float>> each_rob_direct0;

    int distForR2R = 4;
//    if(map_id == 4) distForR2R = 5;
    float ang_big = M_PI/3;
    float ang_small = M_PI/6;
    float ang_range = M_PI/2;
    float w = -1;
    float v = 0;
    for(int cnt_rob = 0; cnt_rob < 4; cnt_rob++)
    {
        if(cnt_rob == 3)
        {
            for(int i = 0; i < 4; i++)
            {
                if(i != cnt_rob)
                {
                    each_rob_dis3.push_back(make_pair(getDistR2R(glo_robots[cnt_rob], glo_robots[i]), i));

                    float body_rob_x =  (glo_robots[i].loc.x-glo_robots[cnt_rob].loc.x)*cos(glo_robots[cnt_rob].direct)+(glo_robots[i].loc.y-glo_robots[cnt_rob].loc.y)*sin(glo_robots[cnt_rob].direct);
                    float body_rob_y = -(glo_robots[i].loc.x-glo_robots[cnt_rob].loc.x)*sin(glo_robots[cnt_rob].direct)+(glo_robots[i].loc.y-glo_robots[cnt_rob].loc.y)*cos(glo_robots[cnt_rob].direct);
                    float r_direct1 = atan2(body_rob_y,body_rob_x);
                    each_rob_direct3.push_back(make_pair(getDistR2R(glo_robots[cnt_rob], glo_robots[i]), r_direct1));
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
        }
        else if(cnt_rob == 2)
        {
            for(int i = 0; i < 4; i++)
            {
                if(i != cnt_rob)
                {
                    each_rob_dis2.push_back(make_pair(getDistR2R(glo_robots[cnt_rob], glo_robots[i]), i));

                    float body_rob_x =  (glo_robots[i].loc.x-glo_robots[cnt_rob].loc.x)*cos(glo_robots[cnt_rob].direct)+(glo_robots[i].loc.y-glo_robots[cnt_rob].loc.y)*sin(glo_robots[cnt_rob].direct);
                    float body_rob_y = -(glo_robots[i].loc.x-glo_robots[cnt_rob].loc.x)*sin(glo_robots[cnt_rob].direct)+(glo_robots[i].loc.y-glo_robots[cnt_rob].loc.y)*cos(glo_robots[cnt_rob].direct);
                    float r_direct1 = atan2(body_rob_y,body_rob_x);
                    each_rob_direct2.push_back(make_pair(getDistR2R(glo_robots[cnt_rob], glo_robots[i]), r_direct1));
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

        }
        else if(cnt_rob == 1)
        {
            for(int i = 0; i < 4; i++)
            {
                if(i != cnt_rob)
                {
                    each_rob_dis1.push_back(make_pair(getDistR2R(glo_robots[cnt_rob], glo_robots[i]), i));

                    float body_rob_x =  (glo_robots[i].loc.x-glo_robots[cnt_rob].loc.x)*cos(glo_robots[cnt_rob].direct)+(glo_robots[i].loc.y-glo_robots[cnt_rob].loc.y)*sin(glo_robots[cnt_rob].direct);
                    float body_rob_y = -(glo_robots[i].loc.x-glo_robots[cnt_rob].loc.x)*sin(glo_robots[cnt_rob].direct)+(glo_robots[i].loc.y-glo_robots[cnt_rob].loc.y)*cos(glo_robots[cnt_rob].direct);
                    float r_direct1 = atan2(body_rob_y,body_rob_x);
                    each_rob_direct1.push_back(make_pair(getDistR2R(glo_robots[cnt_rob], glo_robots[i]), r_direct1));
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

        }
        else if(cnt_rob == 0)
        {
            for(int i = 0; i < 4; i++)
            {
                if(i != cnt_rob)
                {
                    each_rob_dis0.push_back(make_pair(getDistR2R(glo_robots[cnt_rob], glo_robots[i]), i));

                    float body_rob_x =  (glo_robots[i].loc.x-glo_robots[cnt_rob].loc.x)*cos(glo_robots[cnt_rob].direct)+(glo_robots[i].loc.y-glo_robots[cnt_rob].loc.y)*sin(glo_robots[cnt_rob].direct);
                    float body_rob_y = -(glo_robots[i].loc.x-glo_robots[cnt_rob].loc.x)*sin(glo_robots[cnt_rob].direct)+(glo_robots[i].loc.y-glo_robots[cnt_rob].loc.y)*cos(glo_robots[cnt_rob].direct);
                    float r_direct1 = atan2(body_rob_y,body_rob_x);
                    each_rob_direct0.push_back(make_pair(getDistR2R(glo_robots[cnt_rob], glo_robots[i]), r_direct1));
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

        }
    }
    if(robotID == 0)
    {
        for(int i=0; i<each_rob_dis0.size(); i++)
        {
            if(each_rob_direct0[i].first <= distForR2R && (each_rob_direct0[i].second >= -ang_range && each_rob_direct0[i].second <= ang_range))
            {
                if(each_rob_direct0[i].first >= 1.5){
                    if(robotID < each_rob_dis0[i].second && each_rob_direct0[i].second <= asin(1.1/each_rob_direct0[i].first) && each_rob_direct0[i].second >= -asin(1.1/each_rob_direct0[i].first))
                    {
                        float x_direct,y_direct,body_x_direct,body_y_direct;
                        x_direct = cos(glo_robots[each_rob_dis0[i].second].direct);
                        y_direct = sin(glo_robots[each_rob_dis0[i].second].direct);
                        body_x_direct =  x_direct*cos(glo_robots[robotID].direct) + y_direct*sin(glo_robots[robotID].direct);
                        body_y_direct = -x_direct*sin(glo_robots[robotID].direct) + y_direct*cos(glo_robots[robotID].direct);
                        float body_directB;
                        body_directB = atan2(body_y_direct,body_x_direct);
                        float v_true,v_A;
                        v_true = sqrt(glo_robots[each_rob_dis0[i].second].vel_line.x*glo_robots[each_rob_dis0[i].second].vel_line.x + glo_robots[each_rob_dis0[i].second].vel_line.y*glo_robots[each_rob_dis0[i].second].vel_line.y);
                        v_A = sqrt(glo_robots[robotID].vel_line.x*glo_robots[robotID].vel_line.x + glo_robots[robotID].vel_line.y*glo_robots[robotID].vel_line.y);
                        if(each_rob_direct0[i].second >= 0 && each_rob_direct0[i].second <= ang_range)
                        {
                            if(!(body_directB>=0 && body_directB<=M_PI/2 && v_true>=v_A))
                            {
                                w = -ang_big;
                            }
                        }
                        else
                        {
                            if(!(body_directB<=0 && body_directB>=(-M_PI/2 && v_true>=v_A)))
                            {
                                w = ang_big;
                            }
                        }
                        if(each_rob_direct0[i].first<2.5){
                            if(v_true > v)
                                control(each_rob_dis0[i].second,FORWARD_ACTION,v);
                        }
                    }
                }
                else if(each_rob_direct0[i].first < 1.5){
                    if(robotID < each_rob_dis0[i].second)
                    {
                        if(each_rob_direct0[i].second >= 0 && each_rob_direct0[i].second <= ang_range)
                        {
                            w = -ang_big;
                        }
                        else
                        {
                            w = ang_big;
                        }
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
            if(each_rob_direct1[i].first <= distForR2R && (each_rob_direct1[i].second >= -ang_range && each_rob_direct1[i].second <= ang_range))
            {
                if(each_rob_direct1[i].first >= 1.5 )
                {
                    if(each_rob_direct1[i].second <= asin(1.1/each_rob_direct1[i].first) && each_rob_direct1[i].second >= -asin(1.1/each_rob_direct1[i].first))
                    {
                        if(robotID < each_rob_dis1[i].second)
                        {
                            float x_direct,y_direct,body_x_direct,body_y_direct;
                            x_direct = cos(glo_robots[each_rob_dis1[i].second].direct);
                            y_direct = sin(glo_robots[each_rob_dis1[i].second].direct);
                            body_x_direct =  x_direct*cos(glo_robots[robotID].direct) + y_direct*sin(glo_robots[robotID].direct);
                            body_y_direct = -x_direct*sin(glo_robots[robotID].direct) + y_direct*cos(glo_robots[robotID].direct);
                            float body_directB;
                            body_directB = atan2(body_y_direct,body_x_direct);
                            float v_true,v_A;
                            v_true = sqrt(glo_robots[each_rob_dis1[i].second].vel_line.x*glo_robots[each_rob_dis1[i].second].vel_line.x + glo_robots[each_rob_dis1[i].second].vel_line.y*glo_robots[each_rob_dis1[i].second].vel_line.y);
                            v_A = sqrt(glo_robots[robotID].vel_line.x*glo_robots[robotID].vel_line.x + glo_robots[robotID].vel_line.y*glo_robots[robotID].vel_line.y);
                            if(each_rob_direct1[i].second >= 0 && each_rob_direct1[i].second <= ang_range)
                            {
                                if(!(body_directB>=0 && body_directB<=M_PI/2 && v_true>=v_A))
                                {
                                    w = -ang_big;
                                }
                            }
                            else
                            {
                                if(!(body_directB<=0 && body_directB>=(-M_PI/2 && v_true>=v_A)))
                                {
                                    w = ang_big;
                                }
                            }
                            if(each_rob_direct1[i].first<2.5)
                            {
                                if(v_true > v)
                                    control(each_rob_dis1[i].second,FORWARD_ACTION,v);
                            }
                        }
                        else if(each_rob_dis1[i].second == 0)
                        {
                            if(!(avoid_area(each_rob_dis1[i].second,robotID,distForR2R)))
                            {
                                if(each_rob_direct1[i].second >= 0 && each_rob_direct1[i].second <= ang_range)
                                {
                                    w = -ang_big;
                                }
                                else
                                {
                                    w = ang_big;
                                }
                            }
                        }
                    }
                }
                else if(each_rob_direct0[i].first < 1.5){
                    if(robotID < each_rob_dis0[i].second)
                    {
                        if(each_rob_direct0[i].second >= 0 && each_rob_direct0[i].second <= ang_range)
                        {
                            w = -ang_big;
                        }
                        else
                        {
                            w = ang_big;
                        }
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
            if(each_rob_direct2[i].first <= distForR2R && (each_rob_direct2[i].second >= -ang_range && each_rob_direct2[i].second <= ang_range))
            {
                if(each_rob_direct2[i].first >= 1.5)
                {
                    if((each_rob_direct2[i].second >= -(asin(1.1/each_rob_direct2[i].first)) && (each_rob_direct2[i].second <= asin(1.1/each_rob_direct2[i].first))))
                    {
                        if(robotID < each_rob_dis2[i].second)
                        {
                            float x_direct,y_direct,body_x_direct,body_y_direct;
                            x_direct = cos(glo_robots[each_rob_dis2[i].second].direct);
                            y_direct = sin(glo_robots[each_rob_dis2[i].second].direct);
                            body_x_direct =  x_direct*cos(glo_robots[robotID].direct) + y_direct*sin(glo_robots[robotID].direct);
                            body_y_direct = -x_direct*sin(glo_robots[robotID].direct) + y_direct*cos(glo_robots[robotID].direct);
                            float body_directB;
                            body_directB = atan2(body_y_direct,body_x_direct);
                            float v_true,v_A;
                            v_true = sqrt(glo_robots[each_rob_dis2[i].second].vel_line.x*glo_robots[each_rob_dis2[i].second].vel_line.x + glo_robots[each_rob_dis2[i].second].vel_line.y*glo_robots[each_rob_dis2[i].second].vel_line.y);
                            v_A = sqrt(glo_robots[robotID].vel_line.x*glo_robots[robotID].vel_line.x + glo_robots[robotID].vel_line.y*glo_robots[robotID].vel_line.y);
                            if(each_rob_direct2[i].second >= 0 && each_rob_direct2[i].second <= ang_range)
                            {
                                if(!(body_directB>=0 && body_directB<=M_PI/2 && v_true>=v_A))
                                {
                                    w = -ang_big;
                                }
                            }
                            else
                            {
                                if(!(body_directB<=0 && body_directB>=(-M_PI/2 && v_true>=v_A)))
                                {
                                    w = ang_big;
                                }
                            }
                            if(each_rob_direct2[i].first<2.5)
                            {
                                float v_true;
                                v_true = sqrt(glo_robots[each_rob_dis2[i].second].vel_line.x*glo_robots[each_rob_dis2[i].second].vel_line.x + glo_robots[each_rob_dis2[i].second].vel_line.y*glo_robots[each_rob_dis2[i].second].vel_line.y);
                                if(v_true > v)
                                    control(each_rob_dis2[i].second,FORWARD_ACTION,v);
                            }
                        }
                        else if(each_rob_dis2[i].second == 0)
                        {
                            if(!(avoid_area(each_rob_dis2[i].second,robotID,distForR2R)))
                            {
                                if(each_rob_direct2[i].second >= 0 && each_rob_direct2[i].second <= ang_range)
                                {
                                    w = -ang_big;
                                }
                                else
                                {
                                    w = ang_big;
                                }
                            }
                        }
                        else if(each_rob_dis2[i].second == 1)
                        {
                            if(!(avoid_area(each_rob_dis2[i].second,robotID,distForR2R)))
                            {
                                if(each_rob_direct2[i].second >= 0 && each_rob_direct2[i].second <= ang_range)
                                {
                                    w = -ang_big;
                                }
                                else
                                {
                                    w = ang_big;
                                }
                            }
                        }
                    }
                }
                else if(each_rob_direct2[i].first < 1.5){
                    if(robotID < each_rob_dis2[i].second)
                    {
                        if(each_rob_direct2[i].second >= 0 && each_rob_direct2[i].second <= ang_range)
                        {
                            w = -ang_big;
                        }
                        else
                        {
                            w = ang_big;
                        }
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
            if(each_rob_direct3[i].first <= distForR2R && (each_rob_direct3[i].second >= -ang_range && each_rob_direct3[i].second <= ang_range))
            {
                if(each_rob_direct3[i].first >= 1.5)
                {
                    if((each_rob_direct3[i].second >= -(asin(1.1/each_rob_direct3[i].first)) && each_rob_direct3[i].second <= (asin(1.1/each_rob_direct3[i].first))))
                    {
                        if(robotID < each_rob_dis3[i].second)
                        {
                            float x_direct,y_direct,body_x_direct,body_y_direct;
                            x_direct = cos(glo_robots[each_rob_dis3[i].second].direct);
                            y_direct = sin(glo_robots[each_rob_dis3[i].second].direct);
                            body_x_direct =  x_direct*cos(glo_robots[robotID].direct) + y_direct*sin(glo_robots[robotID].direct);
                            body_y_direct = -x_direct*sin(glo_robots[robotID].direct) + y_direct*cos(glo_robots[robotID].direct);
                            float body_directB;
                            body_directB = atan2(body_y_direct,body_x_direct);
                            float v_true,v_A;
                            v_true = sqrt(glo_robots[each_rob_dis3[i].second].vel_line.x*glo_robots[each_rob_dis3[i].second].vel_line.x + glo_robots[each_rob_dis3[i].second].vel_line.y*glo_robots[each_rob_dis3[i].second].vel_line.y);
                            v_A = sqrt(glo_robots[robotID].vel_line.x*glo_robots[robotID].vel_line.x + glo_robots[robotID].vel_line.y*glo_robots[robotID].vel_line.y);
                            if(each_rob_direct3[i].second >= 0 && each_rob_direct3[i].second <= ang_range)
                            {
                                if(!(body_directB>=0 && body_directB<=M_PI/2 && v_true>=v_A))
                                {
                                    w = -ang_big;
                                }
                            }
                            else
                            {
                                if(!(body_directB<=0 && body_directB>=(-M_PI/2 && v_true>=v_A)))
                                {
                                    w = ang_big;
                                }
                            }
                            if(each_rob_direct3[i].first<2.5)
                            {
                                float v_true;
                                v_true = sqrt(glo_robots[each_rob_dis3[i].second].vel_line.x*glo_robots[each_rob_dis3[i].second].vel_line.x + glo_robots[each_rob_dis3[i].second].vel_line.y*glo_robots[each_rob_dis3[i].second].vel_line.y);
                                if(v_true > v)
                                    control(each_rob_dis3[i].second,FORWARD_ACTION,v);
                            }
                        }
                        else if(each_rob_dis3[i].second == 0)
                        {
                            if(!(avoid_area(each_rob_dis3[i].second,robotID,distForR2R)))
                            {
                                if(each_rob_direct3[i].second >= 0 && each_rob_direct3[i].second <= ang_range)
                                {
                                    w = -ang_big;
                                }
                                else
                                {
                                    w = ang_big;
                                }
                                if(each_rob_direct3[i].first<2.5)
                                {
                                    float v_true;
                                    v_true = sqrt(glo_robots[each_rob_dis3[i].second].vel_line.x*glo_robots[each_rob_dis3[i].second].vel_line.x + glo_robots[each_rob_dis3[i].second].vel_line.y*glo_robots[each_rob_dis3[i].second].vel_line.y);
                                    if(v_true > v)
                                        control(each_rob_dis3[i].second,FORWARD_ACTION,v);
                                }
                            }
                        }
                        else if(each_rob_dis3[i].second == 1)
                        {
                            if(!(avoid_area(each_rob_dis3[i].second,robotID,distForR2R)))
                            {
                                if(each_rob_direct3[i].second >= 0 && each_rob_direct3[i].second <= ang_range)
                                {
                                    w = -ang_big;
                                }
                                else
                                {
                                    w = ang_big;
                                }
                                if(each_rob_direct3[i].first<2.5)
                                {
                                    float v_true;
                                    v_true = sqrt(glo_robots[each_rob_dis3[i].second].vel_line.x*glo_robots[each_rob_dis3[i].second].vel_line.x + glo_robots[each_rob_dis3[i].second].vel_line.y*glo_robots[each_rob_dis3[i].second].vel_line.y);
                                    if(v_true > v)
                                        control(each_rob_dis3[i].second,FORWARD_ACTION,v);
                                }
                            }
                        }
                        else if(each_rob_dis3[i].second == 2)
                        {
                            if(!(avoid_area(each_rob_dis3[i].second,robotID,distForR2R)))
                            {
                                if(each_rob_direct3[i].second >= 0 && each_rob_direct3[i].second <= ang_range)
                                {
                                    w = -ang_big;
                                }
                                else
                                {
                                    w = ang_big;
                                }
                                if(each_rob_direct3[i].first<2.5)
                                {
                                    float v_true;
                                    v_true = sqrt(glo_robots[each_rob_dis3[i].second].vel_line.x*glo_robots[each_rob_dis3[i].second].vel_line.x + glo_robots[each_rob_dis3[i].second].vel_line.y*glo_robots[each_rob_dis3[i].second].vel_line.y);
                                    if(v_true > v)
                                        control(each_rob_dis3[i].second,FORWARD_ACTION,v);
                                }
                            }
                        }
                    }
                }
                else if(each_rob_direct3[i].first < 1.5)
                {
                    if(robotID < each_rob_dis3[i].second)
                    {
                        if(each_rob_direct3[i].second >= 0 && each_rob_direct3[i].second <= ang_range)
                        {
                            w = -ang_big;
                        }
                        else
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
    int flag=0;//标记机器人在同一个墙角的数量
    for(int cnt_rob = 0; cnt_rob < 4; cnt_rob++){
        float x1,y1;
        x1 = glo_robots[cnt_rob].loc.x;
        y1 = glo_robots[cnt_rob].loc.y;
        if(x1 < 1 && y1 < 1){//左下角
            for(int i = 0; i < 4; i++){
                if(i != cnt_rob){
                    float dis = getDistR2R(glo_robots[cnt_rob],glo_robots[i]);
                    if(dis < 2.5){
                        control(i,FORWARD_ACTION,v);
                    }
                }
            }
        }
        if(x1 > 49 && y1 < 1){//右下角
            for(int i = 0; i < 4; i++){
                if(i != cnt_rob){
                    float dis = getDistR2R(glo_robots[cnt_rob],glo_robots[i]);
                    if(dis < 2.5){
                        control(i,FORWARD_ACTION,v);
                    }
                }
            }
        }
        if(x1 < 1 && y1 > 49){//左上角
            for(int i = 0; i < 4; i++){
                if(i != cnt_rob){
                    float dis = getDistR2R(glo_robots[cnt_rob],glo_robots[i]);
                    if(dis < 2.5){
                        control(i,FORWARD_ACTION,v);
                    }
                }
            }
        }
        if(x1 > 49 && y1 > 49){//右上角
            for(int i = 0; i < 4; i++){
                if(i != cnt_rob){
                    float dis = getDistR2R(glo_robots[cnt_rob],glo_robots[i]);
                    if(dis < 2.5){
                        control(i,FORWARD_ACTION,v);
                    }
                }
            }
        }
    }
    return w;
}

//探测机器人2，是否在机器人1的避障范围
bool System::avoid_area(int robotID1, int robotID2, float distForR2R) {
    //判断ID2是否在ID1的避障检测区域
    char flag = false;
    float x1,x2,y1,y2,direct1,direct2,dis;
    x1 = glo_robots[robotID1].loc.x;
    y1 = glo_robots[robotID1].loc.y;
    x2 = glo_robots[robotID2].loc.x;
    y2 = glo_robots[robotID2].loc.y;
    direct1 = glo_robots[robotID1].direct;
    direct2 = glo_robots[robotID2].direct;
    dis = getDistR2R(glo_robots[robotID1],glo_robots[robotID2]);
    float body_x2,body_y2,r_direct2;
    body_x2 =  (x2-x1)*cos(direct1) + (y2-y1)*sin(direct1);
    body_y2 = -(x2-x1)*sin(direct1) + (y2-y1)*cos(direct1);
    r_direct2 = atan2(body_y2,body_x2);
    if((dis <= distForR2R && dis >= 1.5))
    {
        if((r_direct2 >= -(asin(1.1/dis)) && r_direct2 <= asin(1.1/dis)))
        {
            flag = true;
        }
    }
    else if(dis < 1.5)
    {
        flag = true;
    }
    return flag;
}
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
