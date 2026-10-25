// Export City Sample's native lane geometry for reproducible ProjectAirSim routes.
#include "Engine/World.h"
#include "EngineUtils.h"
#include "HAL/IConsoleManager.h"
#include "JsonObjectConverter.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonSerializer.h"
#include "ZoneGraphData.h"

static void ExportAGZoneGraph(UWorld* World)
{
	if (!World)
	{
		UE_LOG(LogTemp, Error, TEXT("AG.ZoneGraph.Export: no game world"));
		return;
	}

	TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
	Root->SetStringField(TEXT("map"), World->GetMapName());
	Root->SetStringField(TEXT("units"), TEXT("Unreal centimeters; x/y horizontal, z up"));
	TArray<TSharedPtr<FJsonValue>> DataSets;
	int32 TotalLanes = 0;
	for (TActorIterator<AZoneGraphData> It(World); It; ++It)
	{
		const AZoneGraphData* Data = *It;
		const FZoneGraphStorage& Storage = Data->GetStorage();
		TSharedRef<FJsonObject> DataSet = MakeShared<FJsonObject>();
		DataSet->SetStringField(TEXT("actor"), Data->GetPathName());
		TArray<TSharedPtr<FJsonValue>> Lanes;
		for (int32 Index = 0; Index < Storage.Lanes.Num(); ++Index)
		{
			const FZoneLaneData& Lane = Storage.Lanes[Index];
			if (Lane.PointsBegin < 0 || Lane.PointsEnd > Storage.LanePoints.Num() ||
				Lane.LinksBegin < 0 || Lane.LinksEnd > Storage.LaneLinks.Num())
			{
				continue;
			}
			TSharedRef<FJsonObject> Item = MakeShared<FJsonObject>();
			Item->SetNumberField(TEXT("index"), Index);
			Item->SetNumberField(TEXT("tag_mask"), Lane.Tags.GetValue());
			Item->SetNumberField(TEXT("width_cm"), Lane.Width);
			TArray<TSharedPtr<FJsonValue>> Points;
			for (int32 PointIndex = Lane.PointsBegin; PointIndex < Lane.PointsEnd; ++PointIndex)
			{
				const FVector& Point = Storage.LanePoints[PointIndex];
				TArray<TSharedPtr<FJsonValue>> XYZ;
				XYZ.Add(MakeShared<FJsonValueNumber>(Point.X));
				XYZ.Add(MakeShared<FJsonValueNumber>(Point.Y));
				XYZ.Add(MakeShared<FJsonValueNumber>(Point.Z));
				Points.Add(MakeShared<FJsonValueArray>(MoveTemp(XYZ)));
			}
			Item->SetArrayField(TEXT("points_cm"), MoveTemp(Points));
			TArray<TSharedPtr<FJsonValue>> Links;
			for (int32 LinkIndex = Lane.LinksBegin; LinkIndex < Lane.LinksEnd; ++LinkIndex)
			{
				const FZoneLaneLinkData& Link = Storage.LaneLinks[LinkIndex];
				if (Link.Type == EZoneLaneLinkType::Outgoing)
				{
					Links.Add(MakeShared<FJsonValueNumber>(Link.DestLaneIndex));
				}
			}
			Item->SetArrayField(TEXT("outgoing"), MoveTemp(Links));
			Lanes.Add(MakeShared<FJsonValueObject>(Item));
		}
		TotalLanes += Lanes.Num();
		DataSet->SetArrayField(TEXT("lanes"), MoveTemp(Lanes));
		DataSets.Add(MakeShared<FJsonValueObject>(DataSet));
	}
	Root->SetArrayField(TEXT("datasets"), MoveTemp(DataSets));
	FString Json;
	const TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Json);
	if (!FJsonSerializer::Serialize(Root, Writer))
	{
		UE_LOG(LogTemp, Error, TEXT("AG.ZoneGraph.Export: serialization failed"));
		return;
	}
	const FString Path = FPaths::ProjectSavedDir() / TEXT("AGCitySampleZoneGraph.json");
	if (FFileHelper::SaveStringToFile(Json, *Path))
	{
		UE_LOG(LogTemp, Display, TEXT("AG.ZoneGraph.Export: %d lanes written to %s"), TotalLanes, *Path);
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("AG.ZoneGraph.Export: could not write %s"), *Path);
	}
}

static FAutoConsoleCommandWithWorld AGZoneGraphExportCommand(
	TEXT("AG.ZoneGraph.Export"),
	TEXT("Export loaded City Sample ZoneGraph lanes to Saved/AGCitySampleZoneGraph.json"),
	FConsoleCommandWithWorldDelegate::CreateStatic(&ExportAGZoneGraph));
