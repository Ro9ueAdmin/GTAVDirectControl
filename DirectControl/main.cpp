/*
THIS FILE IS A PART OF GTA V SCRIPT HOOK SDK
http://dev-c.com
(C) Alexander Blade 2015
*/

#include "..\..\ScriptHookV_SDK\inc\main.h"
#include "script.h"
#include "Util/Logger.hpp"
#include <Memory/Versions.h>

BOOL APIENTRY DllMain(HMODULE hInstance, DWORD reason, LPVOID lpReserved) {
    const char* logFile = "./DirectControl.log";
    logger.SetFile(logFile);
    logger.SetMinLevel(DEBUG);

    switch (reason) {
    case DLL_PROCESS_ATTACH: {
        logger.Clear();
        logger.Write(INFO, "DirectControl %s (build %s %s)", "v0.0.0", __DATE__, __TIME__);
        logger.Write(INFO, "Game version " + eGameVersionToString(getGameVersion()));
        logger.Write(INFO, "");
        scriptRegister(hInstance, ScriptMain);
        scriptRegisterAdditionalThread(hInstance, CheatMain);
        break;
    }
    case DLL_PROCESS_DETACH: {
        scriptUnregister(hInstance);
        break;
    }
    }
	return TRUE;
}
