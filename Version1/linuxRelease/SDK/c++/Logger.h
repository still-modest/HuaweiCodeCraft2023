#ifndef __LOGGER_H
#define __LOGGER_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <fcntl.h>
#include <string.h>
#include "Logger.h"

using namespace std;

enum TIMEFORMAT{
	NETLOG = 0,	//	[yyyy\mm\dd hh.MM.ss]
	LOGINLOG=1,	//	mm-dd hh:MM:ss
};
 
 
/******************** 类定义 ********************/
class Logger{
public:
	Logger(string strDir = "./",string filename = "log",int maxfilesize=1,int filecount=0, int timeformat=0);
	~Logger();
 
	void addLog(string log);	//添加日志记录到日志文件
	void fileSizeLimit();		//判断文件大小是否达到限定值
	int getCurrentLogFileSize();//获取当前日志文件的大小
	string getLogFileName();	//获取日志文件名称
	void setMaxFileSize(int);//设置文件最大大小
	void setFileName(string); //设置日志文件名
	void setFileCount(int);	//设置日志文件的个数
private:
	void fileOffset();		//文件名称进行偏移
	bool checkFolderExist(const string &strPath);
	string getCurrentTime();
	
 
private:
	string m_LogFileName;	//文件名
	int m_MaxFileSize;		//文件大小
	int m_FileCount;		//文件个数
	fstream *m_outputFile;	//输出文件流
	string m_strDir;		//目录
	int m_timeFormat;
};


/******************** 函数定义 ********************/
Logger::Logger(string strDir, string filename, int maxsize, int filecount,int timeFormat)
{
	m_strDir = strDir;
	m_LogFileName = m_strDir+string("\\")+filename;
	m_MaxFileSize = maxsize;
	m_FileCount = filecount;
	m_timeFormat = timeFormat;
 
	//判断日志目录是否存在
	if(!checkFolderExist(m_strDir.c_str()))	
	{
		/* _mkdir(m_strDir.c_str()); */
	}
	m_outputFile = new fstream;
	string strname = m_LogFileName+".txt";
 	m_outputFile->open(strname,ofstream::out|ofstream::app);	//打开日志文件
   	bool b=m_outputFile->is_open();
 
 
}
Logger::~Logger()
{
	if(m_outputFile)
		delete m_outputFile;
}
//********************************
//函数名：Logger::checkFolderExist
//描  述：测试目录是否存在
//参  数：strPath 目录名
//返回值：存在返回真
//*************************************
bool Logger::checkFolderExist( const string  & strPath)  
{  
	return true;
	/*
	if(_access(strPath.data(),0) == 0)
		return true;
	else
		return false;
	*/
}  
//********************************
//函数名：Logger::addLog
//描  述：向文件中添加日志信息
//参  数 log 为信息内容
//返回值：void
//*************************************
void Logger::addLog(string log)
{
	string currentTime = getCurrentTime(); //获取本地时间
	if(m_timeFormat == NETLOG)
		*m_outputFile<<"["<<currentTime<<"] "<<log<<endl;
	else
		*m_outputFile<<currentTime<<" "<<log<<endl;
	//判断文件大小
	fileSizeLimit();
}
//********************************
//函数名：Logger::fileSizeLimit
//描  述：判断文件大小是否达到最大值
//参  数：无
//返回值：void
//*************************************
void Logger::fileSizeLimit() 
{
	int filesize = getCurrentLogFileSize();
	if(filesize>=m_MaxFileSize*1024)
		fileOffset();
 
} 
//********************************
//函数名：Logger::fileOffset
//描  述：实现文件名的偏移
//参  数：无
//返回值：void
//*************************************
void Logger::fileOffset()
{
	m_outputFile->close();	//关闭当前文件
	char filename[100]={0};
	char newfilename[100] = {0};
	for(int i = m_FileCount-1;i > 0;i--)
	{
		memset(filename,0,100);
		sprintf(filename,"%s%d.txt",m_LogFileName.data(),i);
		if(checkFolderExist(filename))  //存在
		{
			if(i == m_FileCount-1)
			{
				remove(filename);//删除文件
				continue;
			}
			//文件名序号向后偏移
			memset(newfilename,0,100);
			sprintf(newfilename,"%s%d.txt",m_LogFileName.data(),i+1);
			rename(filename,newfilename);
		}
	}
	memset(filename,0,100);
	sprintf(filename,"%s.txt",m_LogFileName.data());
	sprintf(newfilename,"%s%d.txt",m_LogFileName.data(),1);
	rename(filename,newfilename);
	m_outputFile->open(filename,ofstream::out|ofstream::app);	//打开日志文件
 }
 
//********************************
//函数名：Logger::getCurrentLogFileSize
//描  述：计算当前日记文件的大小
//参  数：无
//返回值：文件大小（KB）
//*************************************
int Logger::getCurrentLogFileSize()
{
	long long filepos = m_outputFile->tellp(); //保存当前文件位置
	m_outputFile->seekp(0,ios_base::end);			//移动到文件尾
	long long filesize = m_outputFile->tellp();	
	m_outputFile->seekp(filepos,ios_base::beg);		//恢复文件位置
	return filesize/1024;
	
}
//获取文件名
string Logger::getLogFileName()
{
	return m_LogFileName+".txt";
}
//设置文件个数
void Logger::setFileCount(int count)
{
	m_FileCount = count;
}
//设置文件名
void Logger::setFileName(string filename)
{
	m_LogFileName = m_strDir+string("\\")+filename;
}
//设置文件大小
void Logger::setMaxFileSize(int maxsize)
{
	m_MaxFileSize = maxsize;
}
 
//********************************
//函数名：Logger::getCurrentTime
//描  述：获取本地时间
//返回值：时间字符串
//*************************************
string Logger::getCurrentTime()
{
	time_t seconds = time(NULL);	//获取时间
	struct tm *p;
	p = localtime(&seconds);//获取本地时间
	char strTime[100] = {0};
	if(m_timeFormat == NETLOG)
		sprintf(strTime,"%d\\%d\\%d %d.%d.%d",1900+p->tm_year,1+p->tm_mon,p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec);
	else
		sprintf(strTime,"%02d-%02d %02d:%02d:%02d",1+p->tm_mon,p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec);
	return string(strTime);
}
#endif