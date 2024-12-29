// Copyright (c) 2022 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "WheeledVehicleMovementComponentNW.h"
#include "PhysicsPublic.h"
#include "PhysXPublic.h"
#include "PhysXVehicleManager.h"
#include "Components/PrimitiveComponent.h"
#include "Logging/MessageLog.h"

/**
 * Constructor: Initializes the wheeled vehicle movement component with default PhysX engine and transmission values.
 */
UWheeledVehicleMovementComponentNW::UWheeledVehicleMovementComponentNW(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    // Initialize engine setup with default PhysX engine data.
    PxVehicleEngineData DefEngineData;
    EngineSetup.MOI = DefEngineData.mMOI;
    EngineSetup.MaxRPM = OmegaToRPM(DefEngineData.mMaxOmega);
    EngineSetup.DampingRateFullThrottle = DefEngineData.mDampingRateFullThrottle;
    EngineSetup.DampingRateZeroThrottleClutchEngaged = DefEngineData.mDampingRateZeroThrottleClutchEngaged;
    EngineSetup.DampingRateZeroThrottleClutchDisengaged = DefEngineData.mDampingRateZeroThrottleClutchDisengaged;

    // Convert PhysX torque curve data to Unreal engine format.
    FRichCurve* TorqueCurveData = EngineSetup.TorqueCurve.GetRichCurve();
    for (PxU32 KeyIdx = 0; KeyIdx < DefEngineData.mTorqueCurve.getNbDataPairs(); ++KeyIdx)
    {
        float Input = DefEngineData.mTorqueCurve.getX(KeyIdx) * EngineSetup.MaxRPM;
        float Output = DefEngineData.mTorqueCurve.getY(KeyIdx) * DefEngineData.mPeakTorque;
        TorqueCurveData->AddKey(Input, Output);
    }

    // Initialize clutch and transmission setup with default PhysX data.
    PxVehicleClutchData DefClutchData;
    TransmissionSetup.ClutchStrength = DefClutchData.mStrength;

    PxVehicleGearsData DefGearSetup;
    TransmissionSetup.GearSwitchTime = DefGearSetup.mSwitchTime;
    TransmissionSetup.ReverseGearRatio = DefGearSetup.mRatios[PxVehicleGearsData::eREVERSE];
    TransmissionSetup.FinalRatio = DefGearSetup.mFinalRatio;

    PxVehicleAutoBoxData DefAutoBoxSetup;
    TransmissionSetup.NeutralGearUpRatio = DefAutoBoxSetup.mUpRatios[PxVehicleGearsData::eNEUTRAL];
    TransmissionSetup.GearAutoBoxLatency = DefAutoBoxSetup.getLatency();
    TransmissionSetup.bUseGearAutoBox = true;

    // Populate forward gears with default ratios and gear thresholds.
    for (uint32 i = PxVehicleGearsData::eFIRST; i < DefGearSetup.mNbRatios; ++i)
    {
        FVehicleNWGearData GearData;
        GearData.DownRatio = DefAutoBoxSetup.mDownRatios[i];
        GearData.UpRatio = DefAutoBoxSetup.mUpRatios[i];
        GearData.Ratio = DefGearSetup.mRatios[i];
        TransmissionSetup.ForwardGears.Add(GearData);
    }

    // Initialize the steering speed curve to scale steering input with vehicle speed.
    FRichCurve* SteeringCurveData = SteeringCurve.GetRichCurve();
    SteeringCurveData->AddKey(0.0f, 1.0f);
    SteeringCurveData->AddKey(20.0f, 0.9f);
    SteeringCurveData->AddKey(60.0f, 0.8f);
    SteeringCurveData->AddKey(120.0f, 0.7f);

    // Default to 4 wheels and initialize wheel and differential setups.
    const int32 NbrWheels = 4;
    WheelSetups.SetNum(NbrWheels);
    DifferentialSetup.SetNum(NbrWheels);

    // Set the default idle brake input.
    IdleBrakeInput = 10;
}

#if WITH_EDITOR
/**
 * PostEditChangeProperty: Ensures valid editor property changes for gear ratios and steering curves.
 */
void UWheeledVehicleMovementComponentNW::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);
    const FName PropertyName = PropertyChangedEvent.Property ? PropertyChangedEvent.Property->GetFName() : NAME_None;

    if (PropertyName == TEXT("DownRatio"))
    {
        // Ensure DownRatio does not exceed UpRatio for all forward gears.
        for (int32 GearIdx = 0; GearIdx < TransmissionSetup.ForwardGears.Num(); ++GearIdx)
        {
            FVehicleNWGearData& GearData = TransmissionSetup.ForwardGears[GearIdx];
            GearData.DownRatio = FMath::Min(GearData.DownRatio, GearData.UpRatio);
        }
    }
    else if (PropertyName == TEXT("UpRatio"))
    {
        // Ensure UpRatio is not less than DownRatio for all forward gears.
        for (int32 GearIdx = 0; GearIdx < TransmissionSetup.ForwardGears.Num(); ++GearIdx)
        {
            FVehicleNWGearData& GearData = TransmissionSetup.ForwardGears[GearIdx];
            GearData.UpRatio = FMath::Max(GearData.DownRatio, GearData.UpRatio);
        }
    }
    else if (PropertyName == TEXT("SteeringCurve"))
    {
        // Clamp steering curve values between 0 and 1.
        TArray<FRichCurveKey> SteerKeys = SteeringCurve.GetRichCurve()->GetCopyOfKeys();
        for (int32 KeyIdx = 0; KeyIdx < SteerKeys.Num(); ++KeyIdx)
        {
            float NewValue = FMath::Clamp(SteerKeys[KeyIdx].Value, 0.0f, 1.0f);
            SteeringCurve.GetRichCurve()->UpdateOrAddKey(SteerKeys[KeyIdx].Time, NewValue);
        }
    }
}
#endif

/**
 * Helper function to configure PhysX differential data based on Unreal setup.
 */
static void GetVehicleDifferentialNWSetup(const TArray<FVehicleNWWheelDifferentialData>& Setup, PxVehicleDifferentialNWData& PxSetup)
{
    for (int32 i = 0; i < Setup.Num(); ++i)
    {
        PxSetup.setDrivenWheel(i, Setup[i].bDriven);
    }
}

/**
 * FindPeakTorque: Calculates the peak torque from the torque curve.
 */
float FVehicleNWEngineData::FindPeakTorque() const
{
    float PeakTorque = 0.0f;
    TArray<FRichCurveKey> TorqueKeys = TorqueCurve.GetRichCurveConst()->GetCopyOfKeys();
    for (int32 KeyIdx = 0; KeyIdx < TorqueKeys.Num(); ++KeyIdx)
    {
        FRichCurveKey& Key = TorqueKeys[KeyIdx];
        PeakTorque = FMath::Max(PeakTorque, Key.Value);
    }
    return PeakTorque;
}

/**
 * Configures PhysX engine data based on Unreal engine setup.
 */
static void GetVehicleEngineSetup(const FVehicleNWEngineData& Setup, PxVehicleEngineData& PxSetup)
{
    PxSetup.mMOI = M2ToCm2(Setup.MOI);
    PxSetup.mMaxOmega = RPMToOmega(Setup.MaxRPM);
    PxSetup.mDampingRateFullThrottle = M2ToCm2(Setup.DampingRateFullThrottle);
    PxSetup.mDampingRateZeroThrottleClutchEngaged = M2ToCm2(Setup.DampingRateZeroThrottleClutchEngaged);
    PxSetup.mDampingRateZeroThrottleClutchDisengaged = M2ToCm2(Setup.DampingRateZeroThrottleClutchDisengaged);

    float PeakTorque = Setup.FindPeakTorque(); // In Nm
    PxSetup.mPeakTorque = M2ToCm2(PeakTorque); // Convert Nm to PhysX units.

    // Convert Unreal torque curve to normalized PhysX format.
    PxSetup.mTorqueCurve.clear();
    TArray<FRichCurveKey> TorqueKeys = Setup.TorqueCurve.GetRichCurveConst()->GetCopyOfKeys();
    int32 NumTorqueCurveKeys = FMath::Min<int32>(TorqueKeys.Num(), PxVehicleEngineData::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES);
    for (int32 KeyIdx = 0; KeyIdx < NumTorqueCurveKeys; ++KeyIdx)
    {
        FRichCurveKey& Key = TorqueKeys[KeyIdx];
        PxSetup.mTorqueCurve.addPair(FMath::Clamp(Key.Time / Setup.MaxRPM, 0.0f, 1.0f), Key.Value / PeakTorque);
    }
}

// Additional functions continue...
