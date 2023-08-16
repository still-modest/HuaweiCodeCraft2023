#include <iostream>
#include "Logger.h"
#include "Manager.h"
using namespace std;


int main() {
    Logger myLog("text", "log", 1, 10, 0);
    myLog.addLog("Start!");

    vector<string> frameStr;
    if(!readUntilOK(frameStr)){
        myLog.addLog("Read map failed!");
    }
    Manager sys(frameStr);    // 初始化信息管理器
    sys.statis_table();
    myLog.addLog(to_string(sys.m_robots[0].loc.x) + ", " + to_string(sys.m_robots[0].loc.y));
    myLog.addLog(to_string(sys.m_robots[1].loc.x) + ", " + to_string(sys.m_robots[1].loc.y));
    myLog.addLog(to_string(sys.m_robots[2].loc.x) + ", " + to_string(sys.m_robots[2].loc.y));
    myLog.addLog(to_string(sys.m_robots[3].loc.x) + ", " + to_string(sys.m_robots[3].loc.y));
    
    puts("OK");     // 初始化完毕
    fflush(stdout);

    int frameID;
    do {
        readUntilOK(frameStr);
        if(frameStr.size() == 0)
            break;
        sys.praseFrameStr(frameStr);

        frameID = sys.m_frameID;

        myLog.addLog(to_string(sys.m_frameID));
        myLog.addLog(to_string(sys.m_robots[0].loc.x) + ", " + to_string(sys.m_robots[0].loc.y));
        if(frameID == 1)
        {
            for(int i = 0; i < sys.m_worktables.size(); i++)
            {
                myLog.addLog(to_string(i) + ", " + to_string(sys.m_worktables[i].loc.x) + ", " + to_string(sys.m_worktables[i].loc.y));
            }    
        }
        
        // for(int i = 0; i < sys.m_amount_workTable; i++)
        // {   
            
        //         sys.m_worktables[i].robot_link=4;
            
        // }
        // for(int i = 0; i < 4; i++)
        // {
        //     sys.m_robots[i].target_table=-1;
        // }
        
        sys.statis_enough();
        sys.robot_action();
        sys.robot_to_table();

        sys.doActions();

    }while (frameID != EOF);
    return 0;
}
