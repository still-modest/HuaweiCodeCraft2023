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

    //myLog.addLog(to_string(sys.m_robots[0].loc.x) + ", " + to_string(sys.m_robots[0].loc.y));
    //myLog.addLog(to_string(sys.m_robots[1].loc.x) + ", " + to_string(sys.m_robots[1].loc.y));
    //myLog.addLog(to_string(sys.m_robots[2].loc.x) + ", " + to_string(sys.m_robots[2].loc.y));
    //myLog.addLog(to_string(sys.m_robots[3].loc.x) + ", " + to_string(sys.m_robots[3].loc.y));
    for(int cnt_table = 0; cnt_table < sys.m_amount_workTable; cnt_table++)
    {
        myLog.addLog(to_string(cnt_table) + ", " + to_string(sys.m_worktables[cnt_table].ID) + ", " + to_string(sys.m_worktables[cnt_table].classID) + ", " + to_string(sys.m_worktables[cnt_table].loc.x) + ", " + to_string(sys.m_worktables[cnt_table].loc.y));
    }
    
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
        
       
        for(int i = 0; i < 4; i++)
        {
            sys.m_robots[i].inorder = 0;
        }
        
        if(sys.class7.size() != 0)
        {
            sys.planner();
        }
        
        else
        {
            sys.planner2();
        }
        sys.doActions();
        
    }while (frameID != EOF);
    return 0;
}
