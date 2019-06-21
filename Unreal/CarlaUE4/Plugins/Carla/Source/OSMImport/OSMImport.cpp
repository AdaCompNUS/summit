// Copyright 2017 Mike Fricker. All Rights Reserved.

#include "OSMImport.h"
#include "ModuleManager.h"


class FOSMImportModule : public IModuleInterface
{

public:

	// IModuleInterface interface
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

};


IMPLEMENT_MODULE(FOSMImportModule, OSMImport)



void FOSMImportModule::StartupModule()
{
}


void FOSMImportModule::ShutdownModule()
{
}

