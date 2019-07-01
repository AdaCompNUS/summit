// Copyright 2017 Mike Fricker. All Rights Reserved.

namespace UnrealBuildTool.Rules
{
	public class OSMImport : ModuleRules
	{
    public OSMImport(ReadOnlyTargetRules Target)
      : base(Target)
    {
      PrivatePCHHeaderFile = "OSMImport.h";
      PrivateDependencyModuleNames.AddRange(
          new string[] {
              "Core",
              "CoreUObject",
              "Engine",
              "XmlParser"
          }
      );
      PublicDependencyModuleNames.AddRange(
          new string[] {
              "ProceduralMeshComponent"
          }
      );
    }
  }
}