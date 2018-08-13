/*
THIS FILE IS A PART OF GTA V SCRIPT HOOK SDK
http://dev-c.com
(C) Alexander Blade 2015
*/

#include "..\..\ScriptHookV_SDK\inc\main.h"
#include "script.h"
#include "Util/Logger.hpp"

BOOL APIENTRY DllMain(HMODULE hInstance, DWORD reason, LPVOID lpReserved)
{
	switch (reason)
	{
	case DLL_PROCESS_ATTACH: {
        const std::string logFile = "./DirectControl.log";
        logger.SetFile(logFile);
        logger.SetMinLevel(DEBUG);
        scriptRegister(hInstance, ScriptMain);
        break;

	}
	case DLL_PROCESS_DETACH:
		scriptUnregister(hInstance);
		break;
	}
	return TRUE;
}
