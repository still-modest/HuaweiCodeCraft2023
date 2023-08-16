#include <iostream>
#include "System.h"
using namespace std;


int main() {

    vector<string> frameStr;
    if(!readUntilOK(frameStr)){
    }
    System sys(frameStr);    // 初始化信息管理器
    sys.statis_table();
    
    puts("OK");     // 初始化完毕
    fflush(stdout);

    int frameID;
    do {
        readUntilOK(frameStr);
        if(frameStr.size() == 0)
            break;
        sys.praseFrameStr(frameStr);

        frameID = sys.glo_frameID;

        sys.robot_actionv2();
 //       sys.set_Pointv4(1,20,20);

        sys.doActions();

    }while (frameID != EOF);
    return 0;
}
