//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------


#include <mico/slam/utils/LogManager.h>
#include <iostream>
#include <cassert>

using namespace std;

namespace mico {

	LogManager *LogManager::mSingleton = nullptr;


	const LogManager::ColorHandle LogManager::cTextRed		= "\033[31m";
	const LogManager::ColorHandle LogManager::cTextYellow	= "\033[33m";
	const LogManager::ColorHandle LogManager::cTextBlue		= "\033[34m";
	const LogManager::ColorHandle LogManager::cTextGreen	= "\033[32m";
	const LogManager::ColorHandle LogManager::cTextReset	= "\033[0m";

	//---------------------------------------------------------------------------------------------------------------------
	void LogManager::init(const string _appName) {
		if (!mSingleton)
			mSingleton = new LogManager(_appName);
		else
			mSingleton->warning("Someone tried to reinitialize the Log Manager");
	}

	//---------------------------------------------------------------------------------------------------------------------
	void LogManager::close(){
		delete mSingleton;
	}

	//---------------------------------------------------------------------------------------------------------------------
	LogManager * LogManager::get(){
		assert(mSingleton != nullptr);
		return mSingleton;
	}

	//---------------------------------------------------------------------------------------------------------------------
	void LogManager::message(const std::string & _tag, const std::string & _msg, bool _useCout, ColorHandle _color) {
		double timeSpan = std::chrono::duration<double>(chrono::high_resolution_clock::now() - mInitTime).count();
		std::string logLine = to_string(timeSpan) + "\t [" + _tag + "] " + _msg;
		mSecureGuard.lock();
		mLogFile << logLine + "\n";
		mLogFile.flush();
		mSecureGuard.unlock();
		logLine = _color + logLine + cTextReset + "\n";
		if(_useCout){
			cout << logLine;
			cout.flush();
		}
	}

	//---------------------------------------------------------------------------------------------------------------------
	void LogManager::status(const std::string & _msg, bool _useCout){
		message("STATUS", _msg, _useCout, cTextBlue);
	}

	//---------------------------------------------------------------------------------------------------------------------
	void LogManager::warning(const std::string & _msg, bool _useCout){
		message("WARNING",_msg, _useCout, cTextYellow);
	}

	//---------------------------------------------------------------------------------------------------------------------
	void LogManager::error(const std::string & _msg, bool _useCout){
		message("ERROR", _msg,_useCout, cTextRed);
	}

	//---------------------------------------------------------------------------------------------------------------------
	void LogManager::saveTimeMark(string _tag) {
		mTimeMap[_tag] = std::chrono::high_resolution_clock::now();
	}

	//---------------------------------------------------------------------------------------------------------------------
	double LogManager::measureTimeBetweenMarks(string _tag1, string _tag2) {
		return std::chrono::duration<double>(mTimeMap[_tag1] - mTimeMap[_tag2]).count();
	}

	//---------------------------------------------------------------------------------------------------------------------
	LogManager::LogManager(const std::string _appName) {
		mLogFile.open(_appName + to_string(time(NULL))+".txt");
		mInitTime = chrono::high_resolution_clock::now();
		status("Initialized log");
	}

	//---------------------------------------------------------------------------------------------------------------------
	LogManager::~LogManager() {
		mLogFile.close();
	}
}