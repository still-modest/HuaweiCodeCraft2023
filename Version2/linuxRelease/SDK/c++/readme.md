更新版本：V1.0
更新时间：2023.3.15
更新内容：

void Manager::workClass(Worktable worktable)        //根据工作台类别作相应处理
double Manager::getDistance(Robot m_robot, Worktable m_worktable)               //获取距离基本功能
void Manager::robotTableDistance()              //获取所有机器人和工作台距离，放入容器
bool Manager::set_Point(uint8_t robotID,float x,float y, float rv, float rw, float rtheta)              //输入位置驱动机器人行驶至目标点


更新版本：V1.0
更新时间：2023.3.15
更新内容：

void planner();				//机器人策略安排
void searchClass7();			//查询7类工作台产品及操作
void searchClass6();			//查询6类工作台产品及操作
void searchClass5();			//查询5类工作台产品及操作
void searchClass4();			//查询4类工作台产品及操作
void searchClass3();			//查询3类工作台产品及操作
void searchClass2();			//查询2类工作台产品及操作
void searchClass1();			//查询1类工作台产品及操作


bool set_Pointv4(uint8_t robotID,float ref_x,float ref_y);		控制机器人行驶至目标点
