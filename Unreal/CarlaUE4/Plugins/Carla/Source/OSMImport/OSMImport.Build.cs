using System;
using System.IO;
using UnrealBuildTool;

namespace UnrealBuildTool.Rules
{
	public class OSMImport : ModuleRules
	{
    public OSMImport(ReadOnlyTargetRules Target)
      : base(Target)
    {
      PrivatePCHHeaderFile = "OSMImport.h";

      bEnableUndefinedIdentifierWarnings = false;

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

      string BoostIncludePath = Path.Combine(ModuleDirectory, "../../CarlaDependencies", "include");
      PublicIncludePaths.Add(BoostIncludePath);
      PrivateIncludePaths.Add(BoostIncludePath);
    
      PublicDefinitions.Add("BOOST_NO_EXCEPTIONS");
    }
  }
}
