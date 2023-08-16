/*修改了m_worktable的排序顺序
以级别最高的工作台为中心向四周排序
可作正式版版本使用
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

Logger managerLog("manager", "text", 1, 10, 0);

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

    char inorder = 0; //是否携带指令，0表示没有指令，1表示正在执行指令
    vector<pair<double,uint8_t> > distances;    //机器人与工作台距离 
    bool arrived = false;

    
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
         * @brief 工作台与工作台距离
         * 
         * @param table1 工作台1
         * @param table2 工作台2
         */
        double getDistanceWW(double table1_x, double table1_y, Worktable table2);

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

        /**
         * @brief 对工作台进行第二次排序
         * 
         */
        void secondSortTable();

        /**
         * @brief 机器人策略安排
         * 
         */
        void planner();
        void planner2();


        /**
         * @brief 查询7类工作台产品及操作
         */
        void searchClass7();

        /**
         * @brief 查询6类工作台产品及操作
         */
        void searchClass6();

        /**
         * @brief 查询5类工作台产品及操作
         */
        void searchClass5();

        /**
         * @brief 查询4类工作台产品及操作
         */
        void searchClass4();

        /**
         * @brief 查询3类工作台产品及操作
         */
        void searchClass3();

        /**
         * @brief 查询2类工作台产品及操作
         */
        void searchClass2();

        /**
         * @brief 查询1类工作台产品及操作
         */
        void searchClass1();

        /**
         * @brief 缺少6号原材料
         */
        void lack_material6(int cnt_table);
        void lack_material5(int cnt_table);
        void lack_material4(int cnt_table);
        void lack_material3(int cnt_table);
        void lack_material2(int cnt_table);
        void lack_material1(int cnt_table);

        /**
         * @brief 6类工作台无产品
         */
        void lack_product6();
        void lack_product5();
        void lack_product4();


        /**
         * @brief 存放不同类型的工作台
         */
        vector<uint8_t> class1;
        vector<uint8_t> class2;
        vector<uint8_t> class3;
        vector<uint8_t> class4;
        vector<uint8_t> class5;
        vector<uint8_t> class6;
        vector<uint8_t> class7;
        vector<uint8_t> class8;
        vector<uint8_t> class9;
        
        /**
         * @brief 控制机器人行驶至目标点
         * @param robotID 机器人
         * @param ref_x 工作台x坐标
         * @param ref_y 工作台y坐标
         */
        bool set_Pointv4(uint8_t robotID,float ref_x,float ref_y);


        Robot m_robots[4];  // 四台机器人
        vector<Worktable> m_worktables; // 工作台
        vector<Worktable> temp_worktables; // 临时工作台

        int m_frameID; //帧序号
        uint32_t m_money;   // 当前金钱数
        uint8_t m_amount_workTable; // 工作台数量

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
                temp_worktables.push_back(wTable);
                cnt_wTable++;

                m_amount_workTable = cnt_wTable;
            }
        }
    }
    //对工作台尽心从中间向四周排序
    secondSortTable();

    //统计各类工作台数量，并记录ID
    for(int i=0;i<m_amount_workTable;i++)
    {
        uint8_t t_class = m_worktables[i].classID;
        switch (t_class)
        {
            case 1:class1.push_back(m_worktables[i].ID);
                break;
            case 2:class2.push_back(m_worktables[i].ID);
                break;
            case 3:class3.push_back(m_worktables[i].ID);
                break;
            case 4:class4.push_back(m_worktables[i].ID);
                break;
            case 5:class5.push_back(m_worktables[i].ID);
                break;
            case 6:class6.push_back(m_worktables[i].ID);
                break;
            case 7:class7.push_back(m_worktables[i].ID);
                break;
            case 8:class8.push_back(m_worktables[i].ID);
                break;
            case 9:class9.push_back(m_worktables[i].ID);
                break;
            default:
                break;
        }
    }

    for (int i = 0; i < class7.size(); i++)
    {
        managerLog.addLog(to_string(i) + " , " + to_string(m_worktables[class7[i]].classID) + " , " + to_string(class7[i]) + " , " + to_string(m_worktables[class7[i]].loc.x) + " , " + to_string(m_worktables[class7[i]].loc.y));
        /* code */
    }

    for (int i = 0; i < class5.size(); i++)
    {
        managerLog.addLog(to_string(i) + " , " + to_string(m_worktables[class5[i]].classID) + " , " + to_string(class5[i]) + " , " + to_string(m_worktables[class5[i]].loc.x) + " , " + to_string(m_worktables[class5[i]].loc.y));
        /* code */
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
        temp_worktables[i].last = stoi(obj);
        one_line >> obj;
        temp_worktables[i].sta_material = stoi(obj);
        one_line >> obj;
        temp_worktables[i].sta_produce = stoul(obj);

        for(int cnt_temp = 0; cnt_temp < m_amount_workTable; cnt_temp++)
        {
            for(int cnt_real = 0; cnt_real < m_amount_workTable; cnt_real++)
            {
                if((temp_worktables[cnt_temp].loc.x == m_worktables[cnt_real].loc.x) && (temp_worktables[cnt_temp].loc.y == m_worktables[cnt_real].loc.y))
                {
                    m_worktables[cnt_real].last = temp_worktables[cnt_temp].last;
                    m_worktables[cnt_real].sta_material = temp_worktables[cnt_temp].sta_material;
                    m_worktables[cnt_real].sta_produce = temp_worktables[cnt_temp].sta_produce;
                }
            }
        }
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



/**********获取机器人与工作台距离**********/
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

/**********获取工作台与工作台距离**********/
double Manager::getDistanceWW(double table1_x, double table1_y, Worktable table2)
{
    double distance = 0;
    double table2_x, table2_y;

    table2_x = table2.loc.x;
    table2_y = table2.loc.y;

    distance = sqrt((table2_x-table1_x)*(table2_x-table1_x) + (table2_y-table1_y)*(table2_y-table1_y));
    return distance;
}

/**********获取所有机器人与所有工作台距离**********/
void Manager::robotTableDistance()
{

    for(int i = 0; i < 4; i++)
    {   
        m_robots[i].distances.clear();
        for(int j = 0; j < m_worktables.size(); j++)
        {
           double dis = getDistance(m_robots[i], m_worktables[j]); //获取机器人与每个工作台的距离，存入容器distances中
           uint8_t tableID = m_worktables[j].classID;
           
           m_robots[i].distances.push_back(make_pair(dis, tableID));
           
        }
    }
}


/**********依次由最大的工作台到最小的工作台与四个机器人进行排序**********/
vector<pair<uint8_t, Worktable>> Manager::distanceSort()
{
    
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

/**********对工作台进行第二次排序**********/
void Manager::secondSortTable()
{

    m_worktables = temp_worktables;
    vector<Worktable> temp_temp_worktables = temp_worktables;

    double center_x = 24.75;
    double center_y = 25.25;


    for (int  i = 0; i < temp_temp_worktables.size(); i++)
    {
        for(int j = 0; j < temp_temp_worktables.size()-i-1;j++)
        {
            uint8_t oneclass = temp_temp_worktables[j].classID;
            uint8_t twoclass = temp_temp_worktables[j+1].classID;
            if(oneclass < twoclass)
            {
                Worktable temp;
                temp = temp_temp_worktables[j];
                temp_temp_worktables[j] = temp_temp_worktables[j+1];
                temp_temp_worktables[j+1] = temp;
            }
        }
    }

    center_x = temp_temp_worktables[0].loc.x;
    center_y = temp_temp_worktables[0].loc.y;


    for (int  i = 0; i < m_worktables.size(); i++)
    {
        for(int j = 0; j < m_worktables.size()-i-1;j++)
        {
            double dist1 = getDistanceWW(center_x, center_y, m_worktables[j]);

            double dist2 = getDistanceWW(center_x, center_y, m_worktables[j+1]);
            
            if(dist1 > dist2)
            {
                Worktable temp;
                temp = m_worktables[j];
                m_worktables[j] = m_worktables[j+1];
                m_worktables[j+1] = temp;
            }
        }
    }
    for(int cnt_table = 0; cnt_table < m_worktables.size(); cnt_table++)
    {
        m_worktables[cnt_table].ID = cnt_table;
    }
}


/**********机器人策略安排**********/
void Manager::planner()
{
    for(int cnt_robot = 0; cnt_robot < 4; cnt_robot++)          //先判断一次每个机器人身上是否拥有7号产品，有则立即去卖到8号或者9号工作台
    {
        if(m_robots[cnt_robot].objectID == 7 && m_robots[cnt_robot].inorder == 0)
        {   
            m_robots[cnt_robot].inorder =1;
            for(int cnt_table = 0; cnt_table < m_worktables.size(); cnt_table++)
            {
                if(m_worktables[cnt_table].classID == 8 || m_worktables[cnt_table].classID == 9)
                {
                    managerLog.addLog(to_string(m_robots[cnt_robot].ID) + ", " + to_string(m_worktables[cnt_table].classID) + ", " + to_string(m_worktables[cnt_table].ID) + ", " + to_string(8));
                    bool flag = false;
                    flag = set_Pointv4(m_robots[cnt_robot].ID, m_worktables[cnt_table].loc.x,  m_worktables[cnt_table].loc.y);
                    if(flag)
                    {
                        control(m_robots[cnt_robot].ID, ACT_SELL);
                         
                    }
                    break;
                }
            }
        }
    }
    
    searchClass7();
    
}

/**********
 机器人策略安排
 如果没有7类工作台
 则使用该计划
**********/
void Manager::planner2()
{
    for(int cnt_robot = 0; cnt_robot < 4; cnt_robot++)          //先判断一次每个机器人身上是否拥有7号产品，有则立即去卖到8号或者9号工作台
    {
        if(m_robots[cnt_robot].objectID == 6 && m_robots[cnt_robot].inorder == 0)
        {   
            m_robots[cnt_robot].inorder = 1;
            for(int cnt_table = 0; cnt_table < m_worktables.size(); cnt_table++)
            {
                if(m_worktables[cnt_table].classID == 9)
                {
                    managerLog.addLog(to_string(m_robots[cnt_robot].ID) + ", " + to_string(m_worktables[cnt_table].classID) + ", " + to_string(m_worktables[cnt_table].ID) + ", " + to_string(8));
                    bool flag = false;
                    flag = set_Pointv4(m_robots[cnt_robot].ID, m_worktables[cnt_table].loc.x,  m_worktables[cnt_table].loc.y);
                    if(flag)
                    {
                        control(m_robots[cnt_robot].ID, ACT_SELL);
                         
                    }
                    break;
                }
            }
        }
    }
    
    searchClass6();
    
}


/**********查询7类工作台产品及操作代码**********/
void Manager::searchClass7()
{   managerLog.addLog(to_string(m_frameID));
    for(int cnt_class = 0;cnt_class < class7.size(); cnt_class++)                   //先判断一遍所有7类工作台是否有产品
    {            

        if(m_worktables[class7[cnt_class]].sta_produce == 1)
        {   
            managerLog.addLog(to_string(class7[cnt_class]));
            double minDist = 100;
            int robot_id = -1;
            for(int cnt_robot = 0; cnt_robot < 4; cnt_robot++)              //这里没有寻找最近的机器人，只是按顺序找未携带物品的机器人
            {
                if(m_robots[cnt_robot].objectID == 0 && m_robots[cnt_robot].inorder == 0)
                {
                    double m_dist = getDistance(m_robots[cnt_robot], m_worktables[class7[cnt_class]]);
                    if(m_dist < minDist)
                    {
                        minDist = m_dist;
                        robot_id = cnt_robot;
                    }
                }
            }
            if(robot_id != -1)
            {
                m_robots[robot_id].inorder = 1;
                managerLog.addLog(to_string(m_robots[robot_id].ID) + ", " + to_string(m_worktables[class7[cnt_class]].classID) + ", " + to_string(m_worktables[class7[cnt_class]].ID) + ", " + to_string(70));
                bool flag = false;
                flag = set_Pointv4(m_robots[robot_id].ID, m_worktables[class7[cnt_class]].loc.x,  m_worktables[class7[cnt_class]].loc.y);
                if(flag)
                {
                    if(m_frameID < 8850)
                        control(m_robots[robot_id].ID, ACT_BUY);
                }
            }
        }
        else if(m_worktables[class7[cnt_class]].sta_produce == 0)
            continue;
    }

    for(int cnt_class = 0;cnt_class < class7.size(); cnt_class++)            //如果所有7类工作台都没有产品
    {
        int cnt_table = class7[cnt_class];
        uint16_t temp_material;
        temp_material = m_worktables[class7[cnt_class]].sta_material;
        if((temp_material & (1 << 6)) == 0)      //表示缺少6号原材料
        {
            lack_material6(cnt_table);
        }
        if((temp_material & (1 << 5)) == 0)      //表示缺少5号原材料
        {
            lack_material5(cnt_table);
        }
        if((temp_material & (1 << 4)) == 0)      //表示缺少4号原材料
        {
            lack_material4(cnt_table);
        }
        if((temp_material &(1 << 6)) && (temp_material &(1 << 5)) && (temp_material &(1 << 4)))            //如果所有材料均有，则跳过这个7类工作台，寻找下一个7类工作台
        {
            continue;
        }
    }

    lack_product6();
    lack_product5();
    lack_product4();
}


/**********查询6类工作台产品及操作代码**********/
void Manager::searchClass6()
{
    for(int cnt_class = 0;cnt_class < class6.size(); cnt_class++)            //先判断一遍所有6类工作台是否有产品
    {
        
        if(m_worktables[class6[cnt_class]].sta_produce == 1)
        {
            double minDist = 100;
            int robot_id = -1;
            for(int cnt_robot = 0; cnt_robot < 4; cnt_robot++)              //这里没有寻找最近的机器人，只是按顺序找未携带物品的机器人
            {
                if(m_robots[cnt_robot].objectID == 0 && m_robots[cnt_robot].inorder == 0)
                {
                    double m_dist = getDistance(m_robots[cnt_robot], m_worktables[class6[cnt_class]]);
                    if(m_dist < minDist)
                    {
                        minDist = m_dist;
                        robot_id = cnt_robot;
                    }
                }
            }
            if(robot_id != -1)
            {
                m_robots[robot_id].inorder = 1;
                managerLog.addLog(to_string(m_robots[robot_id].ID) + ", " + to_string(m_worktables[class6[cnt_class]].classID) + ", " + to_string(m_worktables[class6[cnt_class]].ID) + ", " + to_string(60));
                bool flag = false;
                flag = set_Pointv4(m_robots[robot_id].ID, m_worktables[class6[cnt_class]].loc.x,  m_worktables[class6[cnt_class]].loc.y);
                if(flag)
                {
                    if(m_frameID < 8850)
                        control(m_robots[robot_id].ID, ACT_BUY);
                        
                }
            }
        }
        else if(m_worktables[class6[cnt_class]].sta_produce == 0)
            continue;
        
    }

    lack_product6();
    lack_product5();
    lack_product4();
    
}

/**********查询5类工作台产品及操作代码**********/
void Manager::searchClass5()
{
    for(int cnt_class = 0;cnt_class < class5.size(); cnt_class++)            //先判断一遍所有5类工作台是否有产品
    {
        
        if(m_worktables[class5[cnt_class]].sta_produce == 1)
        {
            double minDist = 100;
            int robot_id = -1;
            for(int cnt_robot = 0; cnt_robot < 4; cnt_robot++)              //这里没有寻找最近的机器人，只是按顺序找未携带物品的机器人
            {
                if(m_robots[cnt_robot].objectID == 0 && m_robots[cnt_robot].inorder == 0)
                {
                    double m_dist = getDistance(m_robots[cnt_robot], m_worktables[class5[cnt_class]]);
                    if(m_dist < minDist)
                    {
                        minDist = m_dist;
                        robot_id = cnt_robot;
                    }
                }
            }
            if(robot_id != -1)
            {
                m_robots[robot_id].inorder = 1;
                managerLog.addLog(to_string(m_robots[robot_id].ID) + ", " + to_string(m_worktables[class5[cnt_class]].classID) + ", " + to_string(m_worktables[class5[cnt_class]].ID) + ", " + to_string(60));
                bool flag = false;
                flag = set_Pointv4(m_robots[robot_id].ID, m_worktables[class5[cnt_class]].loc.x,  m_worktables[class5[cnt_class]].loc.y);
                if(flag)
                {
                    if(m_frameID < 8850)
                    control(m_robots[robot_id].ID, ACT_BUY);
                        
                }
            }
        }
        else if(m_worktables[class5[cnt_class]].sta_produce == 0)
            continue;
        
    }

    lack_product5();
    lack_product4();
}


/**********查询4类工作台产品及操作代码**********/
void Manager::searchClass4()
{
    for(int cnt_class = 0;cnt_class < class4.size(); cnt_class++)            //先判断一遍所有4类工作台是否有产品
    {
        
        if(m_worktables[class4[cnt_class]].sta_produce == 1)
        {
            double minDist = 100;
            int robot_id = -1;
            for(int cnt_robot = 0; cnt_robot < 4; cnt_robot++)              //这里没有寻找最近的机器人，只是按顺序找未携带物品的机器人
            {
                if(m_robots[cnt_robot].objectID == 0 && m_robots[cnt_robot].inorder == 0)
                {
                    double m_dist = getDistance(m_robots[cnt_robot], m_worktables[class4[cnt_class]]);
                    if(m_dist < minDist)
                    {
                        minDist = m_dist;
                        robot_id = cnt_robot;
                    }
                }
            }
            if(robot_id != -1)
            {
                m_robots[robot_id].inorder = 1;
                managerLog.addLog(to_string(m_robots[robot_id].ID) + ", " + to_string(m_worktables[class4[cnt_class]].classID) + ", " + to_string(m_worktables[class4[cnt_class]].ID) + ", " + to_string(60));
                bool flag = false;
                flag = set_Pointv4(m_robots[robot_id].ID, m_worktables[class4[cnt_class]].loc.x,  m_worktables[class4[cnt_class]].loc.y);
                if(flag)
                {
                    if(m_frameID < 8850)
                    control(m_robots[robot_id].ID, ACT_BUY);
                        
                }
            }
        }
        else if(m_worktables[class4[cnt_class]].sta_produce == 0)
            continue;
        
    }

    lack_product4();
    
}

/**********查询3类工作台产品及操作代码**********/
void Manager::searchClass3()
{
    for(int cnt_class = 0;cnt_class < class3.size(); cnt_class++)            //先判断一遍所有3类工作台是否有产品
    {
        
        if(m_worktables[class3[cnt_class]].sta_produce == 1)
        {
            double minDist = 100;
            int robot_id = -1;
            for(int cnt_robot = 0; cnt_robot < 4; cnt_robot++)              //这里没有寻找最近的机器人，只是按顺序找未携带物品的机器人
            {
                if(m_robots[cnt_robot].objectID == 0 && m_robots[cnt_robot].inorder == 0)
                {
                    double m_dist = getDistance(m_robots[cnt_robot], m_worktables[class3[cnt_class]]);
                    if(m_dist < minDist)
                    {
                        minDist = m_dist;
                        robot_id = cnt_robot;
                    }
                }
            }
            if(robot_id != -1)
            {
                m_robots[robot_id].inorder = 1;
                managerLog.addLog(to_string(m_robots[robot_id].ID) + ", " + to_string(m_worktables[class3[cnt_class]].classID) + ", " + to_string(m_worktables[class3[cnt_class]].ID) + ", " + to_string(60));
                bool flag = false;
                flag = set_Pointv4(m_robots[robot_id].ID, m_worktables[class3[cnt_class]].loc.x,  m_worktables[class3[cnt_class]].loc.y);
                if(flag)
                {
                    if(m_frameID < 8850)
                    control(m_robots[robot_id].ID, ACT_BUY);
                        
                }
            }
        }
        else if(m_worktables[class3[cnt_class]].sta_produce == 0)
            continue;
        
    }
}

/**********查询2类工作台产品及操作代码**********/
void Manager::searchClass2()
{
    for(int cnt_class = 0;cnt_class < class2.size(); cnt_class++)            //先判断一遍所有2类工作台是否有产品
    {
        
        if(m_worktables[class2[cnt_class]].sta_produce == 1)
        {
            double minDist = 100;
            int robot_id = -1;
            for(int cnt_robot = 0; cnt_robot < 4; cnt_robot++)              //这里没有寻找最近的机器人，只是按顺序找未携带物品的机器人
            {
                if(m_robots[cnt_robot].objectID == 0 && m_robots[cnt_robot].inorder == 0)
                {
                    double m_dist = getDistance(m_robots[cnt_robot], m_worktables[class2[cnt_class]]);
                    if(m_dist < minDist)
                    {
                        minDist = m_dist;
                        robot_id = cnt_robot;
                    }
                }
            }
            if(robot_id != -1)
            {
                m_robots[robot_id].inorder = 1;
                managerLog.addLog(to_string(m_robots[robot_id].ID) + ", " + to_string(m_worktables[class2[cnt_class]].classID) + ", " + to_string(m_worktables[class2[cnt_class]].ID) + ", " + to_string(60));
                bool flag = false;
                flag = set_Pointv4(m_robots[robot_id].ID, m_worktables[class2[cnt_class]].loc.x,  m_worktables[class2[cnt_class]].loc.y);
                if(flag)
                {
                    if(m_frameID < 8850)
                    control(m_robots[robot_id].ID, ACT_BUY);
                        
                }
            }
        }
        else if(m_worktables[class2[cnt_class]].sta_produce == 0)
            continue;
        
    }
}

/**********查询1类工作台产品及操作代码**********/
void Manager::searchClass1()
{
    for(int cnt_class = 0;cnt_class < class1.size(); cnt_class++)            //先判断一遍所有1类工作台是否有产品
    {
        
        if(m_worktables[class1[cnt_class]].sta_produce == 1)
        {
            double minDist = 100;
            int robot_id = -1;
            for(int cnt_robot = 0; cnt_robot < 4; cnt_robot++)              //这里没有寻找最近的机器人，只是按顺序找未携带物品的机器人
            {
                if(m_robots[cnt_robot].objectID == 0 && m_robots[cnt_robot].inorder == 0)
                {
                    double m_dist = getDistance(m_robots[cnt_robot], m_worktables[class1[cnt_class]]);
                    if(m_dist < minDist)
                    {
                        minDist = m_dist;
                        robot_id = cnt_robot;
                    }
                }
            }
            if(robot_id != -1)
            {
                m_robots[robot_id].inorder = 1;
                managerLog.addLog(to_string(m_robots[robot_id].ID) + ", " + to_string(m_worktables[class1[cnt_class]].classID) + ", " + to_string(m_worktables[class1[cnt_class]].ID) + ", " + to_string(60));
                bool flag = false;
                flag = set_Pointv4(m_robots[robot_id].ID, m_worktables[class1[cnt_class]].loc.x,  m_worktables[class1[cnt_class]].loc.y);
                if(flag)
                {
                    if(m_frameID < 8850)
                    control(m_robots[robot_id].ID, ACT_BUY);
                        
                }
            }
        }
        else if(m_worktables[class1[cnt_class]].sta_produce == 0)
            continue;
        
    }
}

    

/**********缺少6号原材料**********/
void Manager::lack_material6(int cnt_table)
{
    int has_product6 = 0;
    double minDist = 100;
    int robot_id = -1;       
    for(int cnt_robot = 0; cnt_robot < 4; cnt_robot++)          //搜索四个机器人中是否有携带6号原材料的
    {
        if(m_robots[cnt_robot].objectID == 6 && m_robots[cnt_robot].inorder == 0)
        {
            double m_dist = getDistance(m_robots[cnt_robot], m_worktables[cnt_table]);
            if(m_dist < minDist)
            {
                minDist = m_dist;
                robot_id = cnt_robot;
            }
            has_product6 = 1;
        }
    }
    if(robot_id != -1)
    {
        m_robots[robot_id].inorder = 1;
        managerLog.addLog(to_string(m_robots[robot_id].ID) + ", " + to_string(m_worktables[cnt_table].classID) + ", " + to_string(m_worktables[cnt_table].ID) + ", " + to_string(76));
        bool flag = false;
        flag = set_Pointv4(m_robots[robot_id].ID, m_worktables[cnt_table].loc.x,  m_worktables[cnt_table].loc.y);
        if(flag)
        {
            control(m_robots[robot_id].ID, ACT_SELL);     
        }
    }
    
    if(has_product6 == 0)            //如果四个机器人都没有6号原材料
    {
        /*调用6类工作台代码*/
        searchClass6();
    }
}

/**********缺少5号原材料**********/
void Manager::lack_material5(int cnt_table)
{
    int has_product5 = 0;
    double minDist = 100;
    int robot_id = -1;        
    for(int cnt_robot = 0; cnt_robot < 4; cnt_robot++)          //搜索四个机器人中是否有携带5号原材料的
    {
        if(m_robots[cnt_robot].objectID == 5 && m_robots[cnt_robot].inorder == 0)
        {
            double m_dist = getDistance(m_robots[cnt_robot], m_worktables[cnt_table]);
            if(m_dist < minDist)
            {
                minDist = m_dist;
                robot_id = cnt_robot;
            }
            has_product5 = 1;
        }
    }
    if(robot_id != -1)
    {
        m_robots[robot_id].inorder = 1;
        managerLog.addLog(to_string(m_robots[robot_id].ID) + ", " + to_string(m_worktables[cnt_table].classID) + ", " + to_string(m_worktables[cnt_table].ID) + ", " + to_string(75));
        bool flag = false;
        flag = set_Pointv4(m_robots[robot_id].ID, m_worktables[cnt_table].loc.x,  m_worktables[cnt_table].loc.y);
        if(flag)
        {
            control(m_robots[robot_id].ID, ACT_SELL);
                
        }
    }
    if(has_product5 == 0)            //如果四个机器人都没有5号原材料
    {
        /*调用5类工作台代码*/
        searchClass5();
    }
}

/**********缺少4号原材料**********/
void Manager::lack_material4(int cnt_table)
{
    int has_product4 = 0;
    double minDist = 100;
    int robot_id = -1;     
    for(int cnt_robot = 0; cnt_robot < 4; cnt_robot++)          //搜索四个机器人中是否有携带4号原材料的
    {
        if(m_robots[cnt_robot].objectID == 4 && m_robots[cnt_robot].inorder == 0)
        {
            double m_dist = getDistance(m_robots[cnt_robot], m_worktables[cnt_table]);
            if(m_dist < minDist)
            {
                minDist = m_dist;
                robot_id = cnt_robot;
            }
            has_product4 = 1;
            
        }
    }
    if(robot_id != -1)
    {
        m_robots[robot_id].inorder = 1;
        managerLog.addLog(to_string(m_robots[robot_id].ID) + ", " + to_string(m_worktables[cnt_table].classID) + ", " + to_string(m_worktables[cnt_table].ID) + ", " + to_string(74));
        bool flag = false;
        flag = set_Pointv4(m_robots[robot_id].ID, m_worktables[cnt_table].loc.x,  m_worktables[cnt_table].loc.y);
        if(flag)
        {
            control(m_robots[robot_id].ID, ACT_SELL);
                
        }
    }
    if(has_product4 == 0)            //如果四个机器人都没有4号原材料
    {
        /*调用4类工作台代码*/
        searchClass4();
    }
}

/**********缺少3号原材料**********/
void Manager::lack_material3(int cnt_table)
{
    int has_product3 = 0;        
    double minDist = 100;
    int robot_id = -1;     
    for(int cnt_robot = 0; cnt_robot < 4; cnt_robot++)          //搜索四个机器人中是否有携带3号原材料的
    {
        if(m_robots[cnt_robot].objectID == 3 && m_robots[cnt_robot].inorder == 0)
        {
            double m_dist = getDistance(m_robots[cnt_robot], m_worktables[cnt_table]);
            if(m_dist < minDist)
            {
                minDist = m_dist;
                robot_id = cnt_robot;
            }
            has_product3 = 1;
            
        }
    }
    if(robot_id != -1)
    {
        m_robots[robot_id].inorder = 1;
        managerLog.addLog(to_string(m_robots[robot_id].ID) + ", " + to_string(m_worktables[cnt_table].classID) + ", " + to_string(m_worktables[cnt_table].ID) + ", " + to_string(74));
        bool flag = false;
        flag = set_Pointv4(m_robots[robot_id].ID, m_worktables[cnt_table].loc.x,  m_worktables[cnt_table].loc.y);
        if(flag)
        {
            control(m_robots[robot_id].ID, ACT_SELL);
                
        }
    }
    if(has_product3 == 0)            //如果四个机器人都没有3号原材料
    {
        /*调用3类工作台代码*/
        searchClass3();
    }
}

/**********缺少2号原材料**********/
void Manager::lack_material2(int cnt_table)
{
    int has_product2 = 0;        
    double minDist = 100;
    int robot_id = -1;     
    for(int cnt_robot = 0; cnt_robot < 4; cnt_robot++)          //搜索四个机器人中是否有携带2号原材料的
    {
        if(m_robots[cnt_robot].objectID == 2 && m_robots[cnt_robot].inorder == 0)
        {
            double m_dist = getDistance(m_robots[cnt_robot], m_worktables[cnt_table]);
            if(m_dist < minDist)
            {
                minDist = m_dist;
                robot_id = cnt_robot;
            }
            has_product2 = 1;
            
        }
    }
    if(robot_id != -1)
    {
        m_robots[robot_id].inorder = 1;
        managerLog.addLog(to_string(m_robots[robot_id].ID) + ", " + to_string(m_worktables[cnt_table].classID) + ", " + to_string(m_worktables[cnt_table].ID) + ", " + to_string(74));
        bool flag = false;
        flag = set_Pointv4(m_robots[robot_id].ID, m_worktables[cnt_table].loc.x,  m_worktables[cnt_table].loc.y);
        if(flag)
        {
            control(m_robots[robot_id].ID, ACT_SELL);
                
        }
    }
    if(has_product2 == 0)            //如果四个机器人都没有2号原材料
    {
        /*调用2类工作台代码*/
        searchClass2();
    }
}
/**********缺少1号原材料**********/
void Manager::lack_material1(int cnt_table)
{
    int has_product1 = 0;        
    double minDist = 100;
    int robot_id = -1;     
    for(int cnt_robot = 0; cnt_robot < 4; cnt_robot++)          //搜索四个机器人中是否有携带1号原材料的
    {
        if(m_robots[cnt_robot].objectID == 1 && m_robots[cnt_robot].inorder == 0)
        {
            double m_dist = getDistance(m_robots[cnt_robot], m_worktables[cnt_table]);
            if(m_dist < minDist)
            {
                minDist = m_dist;
                robot_id = cnt_robot;
            }
            has_product1 = 1;
            
        }
    }
    if(robot_id != -1)
    {
        m_robots[robot_id].inorder = 1;
        managerLog.addLog(to_string(m_robots[robot_id].ID) + ", " + to_string(m_worktables[cnt_table].classID) + ", " + to_string(m_worktables[cnt_table].ID) + ", " + to_string(74));
        bool flag = false;
        flag = set_Pointv4(m_robots[robot_id].ID, m_worktables[cnt_table].loc.x,  m_worktables[cnt_table].loc.y);
        if(flag)
        {
            control(m_robots[robot_id].ID, ACT_SELL);
                
        }
    }
    if(has_product1 == 0)            //如果四个机器人都没有1号原材料
    {
        /*调用三号工作工作台代码*/
        searchClass1();
    }
}

/**********6类工作台缺产品**********/
void Manager::lack_product6()
{
    for(int cnt_class = 0;cnt_class < class6.size(); cnt_class++)            //如果所有6类工作台都没有产品
    {
        
        
        int cnt_table = class6[cnt_class];    
        uint16_t temp_material;
        temp_material = m_worktables[class6[cnt_class]].sta_material;
        if((temp_material & (1 << 3)) == 0)      //表示缺少3号原材料
        {
            lack_material3(cnt_table);
        }
        if((temp_material & (1 << 2)) == 0)      //表示缺少2号原材料
        {
            lack_material2(cnt_table);
        }
        if((temp_material &(1 << 3)) && (temp_material &(1 << 2)))         //如果所有材料均有，则跳过这个6类工作台，寻找下一个6类工作台
        {
                continue;
        }  
        
    }
}

/**********5类工作台缺产品**********/
void Manager::lack_product5()
{
    for(int cnt_class = 0;cnt_class < class5.size(); cnt_class++)            //如果所有5类工作台都没有产品
    {
        
        int cnt_table = class5[cnt_class];    
        uint16_t temp_material;
        temp_material = m_worktables[class5[cnt_class]].sta_material;
        if((temp_material & (1 << 3)) == 0)      //表示缺少3号原材料
        {
            lack_material3(cnt_table);
        }
        if((temp_material & (1 << 1)) == 0)      //表示缺少1号原材料
        {
            lack_material1(cnt_table);
        }
        if((temp_material &(1 << 3)) && (temp_material &(1 << 1)))         //如果所有材料均有，则跳过这个6类工作台，寻找下一个6类工作台
        {
                continue;
        }  
        
    }
}

/**********4类工作台缺产品**********/
void Manager::lack_product4()
{
    for(int cnt_class = 0;cnt_class < class4.size(); cnt_class++)            //如果所有4类工作台都没有产品
    {
        
        int cnt_table = class4[cnt_class];
        uint16_t temp_material;
        temp_material = m_worktables[class4[cnt_class]].sta_material;
        if((temp_material & (1 << 2)) == 0)      //表示缺少2号原材料
        {
            lack_material2(cnt_table);
        }
        if((temp_material & (1 << 1)) == 0)      //表示缺少1号原材料
        {
            lack_material1(cnt_table);
        }
        if((temp_material &(1 << 2)) && (temp_material &(1 << 1)))         //如果所有材料均有，则跳过这个4类工作台，寻找下一个4类工作台
        {
                continue;
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
    if (abs(body_refy)<0.01){//防止奇异跳变
        r_direct=0;
    }
    else
    {
        r_direct=atan2(body_refy,body_refx);
    }
    float v=6.0f;
    float distance = sqrt((ref_y-m_robots[robotID].loc.y)*(ref_y-m_robots[robotID].loc.y)+(ref_x-m_robots[robotID].loc.x)*(ref_x-m_robots[robotID].loc.x));
    if (distance>=2) v=6.0f;
    else v=4.0f;
    if (v>=6) v=6;
    if (v<=-2) v=-2;
    float w=5.0f*(r_direct);
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
    if (abs(m_robots[robotID].loc.x-ref_x)<0.3 && abs(m_robots[robotID].loc.y-ref_y)<0.3){
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



#endif
