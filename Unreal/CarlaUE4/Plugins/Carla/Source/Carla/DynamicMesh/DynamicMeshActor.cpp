#include "DynamicMeshActor.h"
#include "ConstructorHelpers.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"

ADynamicMeshActor::ADynamicMeshActor(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
  MeshComponent = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Mesh"));
  MeshComponent->bUseAsyncCooking = true;
	RootComponent = MeshComponent;
}

void ADynamicMeshActor::SetMaterial(const FString& Material) {
  MeshComponent->SetMaterial(0, Cast<UMaterial>(StaticLoadObject(UMaterial::StaticClass(), NULL, *Material)));
}

void ADynamicMeshActor::SetTriangles(const TArray<FVector>& Triangles) {
  TArray<int> VertexIndices;
  VertexIndices.SetNum(Triangles.Num(), true);
  
  TArray<FVector> Normals;
  Normals.SetNum(Triangles.Num(), true);

  TArray<FVector2D> UV;
  UV.SetNum(Triangles.Num(), true);

  TArray<FProcMeshTangent> Tangents;
  Tangents.SetNum(Triangles.Num(), true);

  for (int I = 0; I < Triangles.Num(); I += 3) {
    VertexIndices[I] = I;
    VertexIndices[I + 1] = I + 1;
    VertexIndices[I + 2] = I + 2;

    Normals[I] = FVector::CrossProduct(Triangles[I + 2] - Triangles[I], Triangles[I + 1] - Triangles[I]).GetSafeNormal();
    Normals[I + 1] = FVector::CrossProduct(Triangles[I] - Triangles[I + 1], Triangles[I + 2] - Triangles[I + 1]).GetSafeNormal();
    Normals[I + 2] = FVector::CrossProduct(Triangles[I + 1] - Triangles[I + 2], Triangles[I] - Triangles[I + 2]).GetSafeNormal();

    UV[I] = FVector2D(0, 0);
    UV[I + 1] = FVector2D(0, 1);
    UV[I + 2] = FVector2D(1, 1);

    Tangents[I].TangentX = (Triangles[I + 1] - Triangles[I]).GetSafeNormal();
    Tangents[I + 1].TangentX = (Triangles[I + 2] - Triangles[I + 1]).GetSafeNormal();
    Tangents[I + 2].TangentX = (Triangles[I] - Triangles[I + 2]).GetSafeNormal();
  }

  MeshComponent->bUseComplexAsSimpleCollision = true;
  MeshComponent->CreateMeshSection_LinearColor(
      0, 
      Triangles, 
      VertexIndices, 
      Normals, 
      UV, 
      {}, 
      Tangents, 
      true);
  MeshComponent->ContainsPhysicsTriMeshData(true);
}
  
/*
 * https://github.com/EpicGames/UnrealEngine/blob/ab237f46dc0eee40263acbacbe938312eb0dffbb/Engine/Source/Runtime/UMG/Private/Blueprint/AsyncTaskDownloadImage.cpp#L61
 * https://wiki.unrealengine.com/Procedural_Materials
 */
void ADynamicMeshActor::SetTileMesh(FVector BoundsMin, FVector BoundsMax, const TArray<uint8>& RawData) {
  // Create mesh.
  TArray<FVector> Triangles;
  // Bottom triangle.
  Triangles.Emplace(BoundsMin.X, BoundsMin.Y, BoundsMin.Z);
  Triangles.Emplace(BoundsMax.X, BoundsMax.Y, BoundsMax.Z);
  Triangles.Emplace(BoundsMax.X, BoundsMin.Y, (BoundsMin.Z + BoundsMax.Z) / 2);

  // Top triangle.
  Triangles.Emplace(BoundsMax.X, BoundsMax.Y, BoundsMax.Z);
  Triangles.Emplace(BoundsMin.X, BoundsMin.Y, BoundsMin.Z);
  Triangles.Emplace(BoundsMin.X, BoundsMax.Y, (BoundsMin.Z + BoundsMax.Z) / 2);

  TArray<int> VertexIndices;
  VertexIndices.SetNum(Triangles.Num(), true);
  
  TArray<FVector> Normals;
  Normals.SetNum(Triangles.Num(), true);

  TArray<FVector2D> UV;
  UV.SetNum(Triangles.Num(), true);

  TArray<FProcMeshTangent> Tangents;
  Tangents.SetNum(Triangles.Num(), true);

  for (int I = 0; I < Triangles.Num(); I += 3) {
    VertexIndices[I] = I;
    VertexIndices[I + 1] = I + 1;
    VertexIndices[I + 2] = I + 2;

    Normals[I] = FVector::CrossProduct(Triangles[I + 2] - Triangles[I], Triangles[I + 1] - Triangles[I]).GetSafeNormal();
    Normals[I + 1] = FVector::CrossProduct(Triangles[I] - Triangles[I + 1], Triangles[I + 2] - Triangles[I + 1]).GetSafeNormal();
    Normals[I + 2] = FVector::CrossProduct(Triangles[I + 1] - Triangles[I + 2], Triangles[I] - Triangles[I + 2]).GetSafeNormal();

    Tangents[I].TangentX = (Triangles[I + 1] - Triangles[I]).GetSafeNormal();
    Tangents[I + 1].TangentX = (Triangles[I + 2] - Triangles[I + 1]).GetSafeNormal();
    Tangents[I + 2].TangentX = (Triangles[I] - Triangles[I + 2]).GetSafeNormal();
  }

  // Bottom triangle
  UV[0] = FVector2D(0, 1);
  UV[1] = FVector2D(1, 0);
  UV[2] = FVector2D(0, 0);
  
  // Top triangle
  UV[3] = FVector2D(1, 0);
  UV[4] = FVector2D(0, 1);
  UV[5] = FVector2D(1, 1);

  MeshComponent->bUseComplexAsSimpleCollision = true;
  MeshComponent->CreateMeshSection_LinearColor(
      0, 
      Triangles, 
      VertexIndices, 
      Normals, 
      UV, 
      {}, 
      Tangents, 
      true);
  MeshComponent->ContainsPhysicsTriMeshData(true);
  
  // Convert into bitmap.
  IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(FName("ImageWrapper"));
  TSharedPtr<IImageWrapper> ImageWrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::JPEG);
  ImageWrapper->SetCompressed(RawData.GetData(), RawData.Num());
  const TArray<uint8>* Data = NULL;
	ImageWrapper->GetRaw(ERGBFormat::BGRA, 8, Data);

  // Create texture from bitmap.
  UTexture2D* Texture2D = UTexture2D::CreateTransient(ImageWrapper->GetWidth(), ImageWrapper->GetHeight(), PF_B8G8R8A8);
  Texture2D->CompressionSettings = TextureCompressionSettings::TC_VectorDisplacementmap;
  Texture2D->SRGB = 0;
  Texture2D->PlatformData->NumSlices = 1;
  Texture2D->NeverStream = true;
  Texture2D->AddToRoot();
  uint8* TextureData = (uint8*)Texture2D->PlatformData->Mips[0].BulkData.Lock(LOCK_READ_WRITE);
  FMemory::Memcpy(TextureData, Data->GetData(), Data->Num());
  Texture2D->PlatformData->Mips[0].BulkData.Unlock();
  Texture2D->UpdateResource();
  
  // Create material instance dynamic.
  UMaterial* Material = Cast<UMaterial>(StaticLoadObject(
        UMaterial::StaticClass(), NULL, 
        TEXT("/Game/Carla/Static/GenericMaterials/Ground/M_Tile")));
  UMaterialInstanceDynamic* MaterialDynamic = UMaterialInstanceDynamic::Create(Material, NULL);
  MaterialDynamic->SetTextureParameterValue(FName("DynamicTexture"), Texture2D);
  MeshComponent->SetMaterial(0, MaterialDynamic);
}
