/*19日最后一个版本
目前最高分
sort()有bug
可作正式版本使用
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
         * @brief 机器人与工作台距离
         * 
         */
        void robotTableDistance();


        /**
         * @brief 依次由最大的工作台到最小的工作台与四个机器人进行排序
         * 
         */
        vector<pair<uint8_t, Worktable>> distanceSort();
        bool set_Pointv4(uint8_t robotID,float ref_x,float ref_y);
        void robot_to_table();
        int find_recenttable(int robot_id);
        void robot_action();
        void statis_enough();
        void statis_table();
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
//        if(m_robots[i].objectID==0)
//        {
//            if(m_worktables[m_robots[i].target_table].classID==1)//即将去买1
//            {
//                table_class1--;
//            }
//            if(m_worktables[m_robots[i].target_table].classID==2)//即将去买2
//            {
//                table_class2--;
//            }
//            if(m_worktables[m_robots[i].target_table].classID==3)//即将去买3
//            {
//                table_class3--;
//            }
//            if(m_worktables[m_robots[i].target_table].classID==4)//即将去买4
//            {
//                table_class4--;
//            }
//            if(m_worktables[m_robots[i].target_table].classID==5)//即将去买5
//            {
//                table_class5--;
//            }
//            if(m_worktables[m_robots[i].target_table].classID==6)//即将去买6
//            {
//                table_class6--;
//            }
//        }
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

    managerLog.addLog("flag: "+ to_string(class_1_enough) + ", " +  to_string(class_2_enough)+" , " +to_string(class_3_enough));

}
int Manager::find_recenttable(int robot_id)
{
    int target_id=-1;

    if(m_robots[robot_id].objectID==0)//如果没有带，此刻空闲可以取
    {
        float dis=10000,dis_temp=10000;
        for(int i=0;i<m_amount_workTable;i++) {
            if (m_worktables[i].classID == 7 && m_worktables[i].sta_produce && m_worktables[i].robot_link >= robot_id && m_frameID >= 8000) {
                return i;
            }
            if ((m_worktables[i].classID == 4 || m_worktables[i].classID == 5 || m_worktables[i].classID == 6 || m_worktables[i].classID == 7) &&
                m_worktables[i].sta_produce && m_worktables[i].robot_link >= robot_id) {
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
                if (dis_temp < dis) {
                    dis = dis_temp;
                    target_id = i;
                }
            }
        }
        if(target_id!=-1)//找到最近的有货的
        {
            return target_id;
        }
        for(int i=0;i<m_amount_workTable;i++){
            if((m_worktables[i].classID==1 || m_worktables[i].classID==2 || m_worktables[i].classID==3) && m_worktables[i].sta_produce && m_worktables[i].robot_link>=robot_id)//如果有货
            {
                if(class_1_enough && m_worktables[i].classID==1) continue;
                if(class_2_enough && m_worktables[i].classID==2) continue;
                if(class_3_enough && m_worktables[i].classID==3) continue;
                if(class_4_enough && m_worktables[i].classID==4) continue;
                if(class_5_enough && m_worktables[i].classID==5) continue;
                if(class_6_enough && m_worktables[i].classID==6) continue;
                dis_temp=sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                        +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y));
                if(dis_temp<dis)
                {
                    dis=dis_temp;
                    target_id=i;
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

        vector<pair<float,int>> dis;
        for(int i=0;i<m_amount_workTable;i++)
        {
            if(m_worktables[i].classID>3)
            {
                if(m_robots[robot_id].objectID==3)
                {
                    if(m_worktables[i].classID==6 || m_worktables[i].classID==5)
                    {
                        if(((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0)&& (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=3))//如果缺货
                        {
                            dis.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                                     +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y)), i));
                        }
                    }
                }
                if(m_robots[robot_id].objectID==1)
                {
                    if(m_worktables[i].classID==4 || m_worktables[i].classID==5)
                    {
                        if(((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0) && (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=1))//如果缺货
                        {
                            dis.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                                     +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y)), i));
                        }
                    }
                }

                if(m_robots[robot_id].objectID==2)
                {
                    if(m_worktables[i].classID==4 || m_worktables[i].classID==6)
                    {
                        if(((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0)&& (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=2))//如果缺货
                        {
                            dis.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
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
                            if(((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0)&& (m_worktables[i].robot_link==4 || m_robots[m_worktables[i].robot_link].objectID!=4))//如果缺货
                            {
                                dis.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
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
                            if(((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0)&& (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=5))//如果缺货
                            {
                                dis.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
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
                            if(((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0)&& (m_worktables[i].robot_link==4|| m_robots[m_worktables[i].robot_link].objectID!=6))//如果缺货
                            {
                                dis.push_back(make_pair(sqrt((m_robots[robot_id].loc.x-m_worktables[i].loc.x)*(m_robots[robot_id].loc.x-m_worktables[i].loc.x)
                                     +(m_robots[robot_id].loc.y-m_worktables[i].loc.y)*(m_robots[robot_id].loc.y-m_worktables[i].loc.y)), i));
                            }
                        }
                    }

                }
                if(m_robots[robot_id].objectID==7)
                {
                    if(m_worktables[i].classID==8)
                    {
                        if(((m_worktables[i].sta_material & (1<<m_robots[robot_id].objectID))==0)&& m_worktables[i].robot_link==4)//如果缺货
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
            sort(dis.begin(),dis.end());
            if(dis.size()==1)
            {
                target_id = dis[0].second;
            }
            else
            {
                    target_temp0 = dis[0].second;
                    target_temp1 = dis[1].second;
                    
                    if(abs(target_temp0-target_temp1)>=25)
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
   for(int i=0;i<4;i++)
   {
       if(m_robots[i].target_table!=-1)//有目标工作台
       {
           set_Pointv4(i,m_worktables[m_robots[i].target_table].loc.x,m_worktables[m_robots[i].target_table].loc.y);
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
bool Manager::set_Pointv4(uint8_t robotID,float ref_x,float ref_y){
    static float last_refx=ref_x;
    static float last_refy=ref_y;
    if (last_refx != ref_x || last_refy != ref_y){
        m_robots[robotID].arrived= false;
    }
    float r_direct=0;
    float body_refx=(ref_x-m_robots[robotID].loc.x)*cos(m_robots[robotID].direct)+(ref_y-m_robots[robotID].loc.y)*sin(m_robots[robotID].direct);
    float body_refy = -(ref_x-m_robots[robotID].loc.x)*sin(m_robots[robotID].direct)+(ref_y-m_robots[robotID].loc.y)*cos(m_robots[robotID].direct);
        r_direct=atan2(body_refy,body_refx);
    float v=6.0f;
    float distance = sqrt((ref_y-m_robots[robotID].loc.y)*(ref_y-m_robots[robotID].loc.y)+(ref_x-m_robots[robotID].loc.x)*(ref_x-m_robots[robotID].loc.x));
    if (distance>=1) v=6.0f;
    else v=1.0f;
    if (v>=6) v=6;
    if (v<=-2) v=-2;
    float w=10.0f*(r_direct);
    if (w>=M_PI) w=M_PI;
    if (w<-M_PI) w=-M_PI;
//    if (m_robots[robotID].loc.x>=49.5||m_robots[robotID].loc.x<=0.5||m_robots[robotID].loc.y>=49.5||m_robots[robotID].loc.y<=0.5){
//        v=0.5f;//靠近墙
//    }
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
