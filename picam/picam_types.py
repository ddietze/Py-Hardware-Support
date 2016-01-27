"""
.. module: drivers/picam_types
  : platform: Windows
.. moduleauthor:: Daniel R. Dietze <daniel.dietze@berkeley.edu>

Defines data types and enums to be used with picam. See PICam user manual for more information.

..
   This file is part of the pyFSRS app.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http://www.gnu.org/licenses/>.

   Copyright 2014-2016 Daniel Dietze <daniel.dietze@berkeley.edu>.
"""
import ctypes

# +++++++ simple data types +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# data types - use these for communication with the picam library!
# to get the value, use XX.value
piint = ctypes.c_int
piflt = ctypes.c_double     # !!!! native floating point is actually double; ctypes.c_float
pibln = ctypes.c_bool
pichar = ctypes.c_char
pibyte = ctypes.c_byte
pibool = ctypes.c_bool
pi8s = ctypes.c_int8
pi8u = ctypes.c_uint8
pi16s = ctypes.c_int16
pi16u = ctypes.c_uint16
pi32s = ctypes.c_int32
pi32u = ctypes.c_uint32
pi64s = ctypes.c_int64
pi64u = ctypes.c_uint64
pivoid = ctypes.c_void_p

# +++++++ enumerations +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# each enumeration has a lookup table of the same name XXLookup which accepts the integer
# value as key and gives the string name as value
PicamError = {"None": 0,
    "UnexpectedError": 4,
    "UnexpectedNullPointer": 3,
    "InvalidPointer": 35,
    "InvalidCount": 39,
    "InvalidOperation": 42,
    "OperationCanceled": 43,
    "LibraryNotInitialized": 1,
    "LibraryAlreadyInitialized": 5,
    "InvalidEnumeratedType": 16,
    "EnumerationValueNotDefined": 17,
    "NotDiscoveringCameras": 18,
    "AlreadyDiscoveringCameras": 19,
    "NoCamerasAvailable": 34,
    "CameraAlreadyOpened": 7,
    "InvalidCameraID": 8,
    "InvalidHandle": 9,
    "DeviceCommunicationFailed": 15,
    "DeviceDisconnected": 23,
    "DeviceOpenElsewhere": 24,
    "InvalidDemoModel": 6,
    "InvalidDemoSerialNumber": 21,
    "DemoAlreadyConnected": 22,
    "DemoNotSupported": 40,
    "ParameterHasInvalidValueType": 11,
    "ParameterHasInvalidConstraintType": 13,
    "ParameterDoesNotExist": 12,
    "ParameterValueIsReadOnly": 10,
    "InvalidParameterValue": 2,
    "InvalidConstraintCategory": 38,
    "ParameterValueIsIrrelevant": 14,
    "ParameterIsNotOnlineable": 25,
    "ParameterIsNotReadable": 26,
    "InvalidParameterValues": 28,
    "ParametersNotCommitted": 29,
    "InvalidAcquisitionBuffer": 30,
    "InvalidReadoutCount": 36,
    "InvalidReadoutTimeOut": 37,
    "InsufficientMemory": 31,
    "AcquisitionInProgress": 20,
    "AcquisitionNotInProgress": 27,
    "TimeOutOccurred": 32,
    "AcquisitionUpdatedHandlerRegistered": 33,
    "NondestructiveReadoutEnabled": 41}
PicamErrorLookup = dict(zip(PicamError.values(), PicamError.keys()))

PicamEnumeratedType = {
    "Error": 1,
    "EnumeratedType": 29,
    "Model": 2,
    "ComputerInterface": 3,
    "DiscoveryAction": 26,
    "HandleType": 27,
    "ValueType": 4,
    "ConstraintType": 5,
    "Parameter": 6,
    "AdcAnalogGain": 7,
    "AdcQuality": 8,
    "CcdCharacteristicsMask": 9,
    "GateTrackingMask": 36,
    "GatingMode": 34,
    "GatingSpeed": 38,
    "EMIccdGainControlMode": 42,
    "IntensifierOptionsMask": 35,
    "IntensifierStatus": 33,
    "ModulationTrackingMask": 41,
    "OrientationMask": 10,
    "OutputSignal": 11,
    "PhosphorType": 39,
    "PhotocathodeSensitivity": 40,
    "PhotonDetectionMode": 43,
    "PixelFormat": 12,
    "ReadoutControlMode": 13,
    "SensorTemperatureStatus": 14,
    "SensorType": 15,
    "ShutterTimingMode": 16,
    "TimeStampsMask": 17,
    "TriggerCoupling": 30,
    "TriggerDetermination": 18,
    "TriggerResponse": 19,
    "TriggerSource": 31,
    "TriggerTermination": 32,
    "ValueAccess": 20,
    "DynamicsMask": 28,
    "ConstraintScope": 21,
    "ConstraintSeverity": 22,
    "ConstraintCategory": 23,
    "RoisConstraintRulesMask": 24,
    "AcquisitionErrorsMask": 25
}
PicamEnumeratedTypeLookup = dict(zip(PicamEnumeratedType.values(), PicamEnumeratedType.keys()))

PicamModel = {
    "PixisSeries": 0,
    "Pixis100Series": 1,
    "Pixis100F": 2,
    "Pixis100B": 6,
    "Pixis100R": 3,
    "Pixis100C": 4,
    "Pixis100BR": 5,
    "Pixis100BExcelon": 54,
    "Pixis100BRExcelon": 55,
    "PixisXO100B": 7,
    "PixisXO100BR": 8,
    "PixisXB100B": 68,
    "PixisXB100BR": 69,
    "Pixis256Series": 26,
    "Pixis256F": 27,
    "Pixis256B": 29,
    "Pixis256E": 28,
    "Pixis256BR": 30,
    "PixisXB256BR": 31,
    "Pixis400Series": 37,
    "Pixis400F": 38,
    "Pixis400B": 40,
    "Pixis400R": 39,
    "Pixis400BR": 41,
    "Pixis400BExcelon": 56,
    "Pixis400BRExcelon": 57,
    "PixisXO400B": 42,
    "PixisXB400BR": 70,
    "Pixis512Series": 43,
    "Pixis512F": 44,
    "Pixis512B": 45,
    "Pixis512BUV": 46,
    "Pixis512BExcelon": 58,
    "PixisXO512F": 49,
    "PixisXO512B": 50,
    "PixisXF512F": 48,
    "PixisXF512B": 47,
    "Pixis1024Series": 9,
    "Pixis1024F": 10,
    "Pixis1024B": 11,
    "Pixis1024BR": 13,
    "Pixis1024BUV": 12,
    "Pixis1024BExcelon": 59,
    "Pixis1024BRExcelon": 60,
    "PixisXO1024F": 16,
    "PixisXO1024B": 14,
    "PixisXO1024BR": 15,
    "PixisXF1024F": 17,
    "PixisXF1024B": 18,
    "PixisXB1024BR": 71,
    "Pixis1300Series": 51,
    "Pixis1300F": 52,
    "Pixis1300F_2": 75,
    "Pixis1300B": 53,
    "Pixis1300BR": 73,
    "Pixis1300BExcelon": 61,
    "Pixis1300BRExcelon": 62,
    "PixisXO1300B": 65,
    "PixisXF1300B": 66,
    "PixisXB1300R": 72,
    "Pixis2048Series": 20,
    "Pixis2048F": 21,
    "Pixis2048B": 22,
    "Pixis2048BR": 67,
    "Pixis2048BExcelon": 63,
    "Pixis2048BRExcelon": 74,
    "PixisXO2048B": 23,
    "PixisXF2048F": 25,
    "PixisXF2048B": 24,
    "Pixis2KSeries": 32,
    "Pixis2KF": 33,
    "Pixis2KB": 34,
    "Pixis2KBUV": 36,
    "Pixis2KBExcelon": 64,
    "PixisXO2KB": 35,
    "QuadroSeries": 100,
    "Quadro4096": 101,
    "Quadro4096_2": 103,
    "Quadro4320": 102,
    "ProEMSeries": 200,
    "ProEM512Series": 203,
    "ProEM512B": 201,
    "ProEM512BK": 205,
    "ProEM512BExcelon": 204,
    "ProEM512BKExcelon": 206,
    "ProEM1024Series": 207,
    "ProEM1024B": 202,
    "ProEM1024BExcelon": 208,
    "ProEM1600Series": 209,
    "ProEM1600xx2B": 212,
    "ProEM1600xx2BExcelon": 210,
    "ProEM1600xx4B": 213,
    "ProEM1600xx4BExcelon": 211,
    "ProEMPlusSeries": 600,
    "ProEMPlus512Series": 603,
    "ProEMPlus512B": 601,
    "ProEMPlus512BK": 605,
    "ProEMPlus512BExcelon": 604,
    "ProEMPlus512BKExcelon": 606,
    "ProEMPlus1024Series": 607,
    "ProEMPlus1024B": 602,
    "ProEMPlus1024BExcelon": 608,
    "ProEMPlus1600Series": 609,
    "ProEMPlus1600xx2B": 612,
    "ProEMPlus1600xx2BExcelon": 610,
    "ProEMPlus1600xx4B": 613,
    "ProEMPlus1600xx4BExcelon": 611,
    "ProEMHSSeries": 1200,
    "ProEMHS512Series": 1201,
    "ProEMHS512B": 1202,
    "ProEMHS512BK": 1207,
    "ProEMHS512BExcelon": 1203,
    "ProEMHS512BKExcelon": 1208,
    "ProEMHS1024Series": 1204,
    "ProEMHS1024B": 1205,
    "ProEMHS1024BExcelon": 1206,
    "PIMax3Series": 300,
    "PIMax31024I": 301,
    "PIMax31024x256": 302,
    "PIMax4Series": 700,
    "PIMax41024ISeries": 703,
    "PIMax41024I": 701,
    "PIMax41024IRF": 704,
    "PIMax41024FSeries": 710,
    "PIMax41024F": 711,
    "PIMax41024FRF": 712,
    "PIMax41024x256Series": 705,
    "PIMax41024x256": 702,
    "PIMax41024x256RF": 706,
    "PIMax42048Series": 716,
    "PIMax42048F": 717,
    "PIMax42048B": 718,
    "PIMax42048FRF": 719,
    "PIMax42048BRF": 720,
    "PIMax4512EMSeries": 708,
    "PIMax4512EM": 707,
    "PIMax4512BEM": 709,
    "PIMax41024EMSeries": 713,
    "PIMax41024EM": 715,
    "PIMax41024BEM": 714,
    "PylonSeries": 400,
    "Pylon100Series": 418,
    "Pylon100F": 404,
    "Pylon100B": 401,
    "Pylon100BR": 407,
    "Pylon100BExcelon": 425,
    "Pylon100BRExcelon": 426,
    "Pylon256Series": 419,
    "Pylon256F": 409,
    "Pylon256B": 410,
    "Pylon256E": 411,
    "Pylon256BR": 412,
    "Pylon400Series": 420,
    "Pylon400F": 405,
    "Pylon400B": 402,
    "Pylon400BR": 408,
    "Pylon400BExcelon": 427,
    "Pylon400BRExcelon": 428,
    "Pylon1024Series": 421,
    "Pylon1024B": 417,
    "Pylon1024BExcelon": 429,
    "Pylon1300Series": 422,
    "Pylon1300F": 406,
    "Pylon1300B": 403,
    "Pylon1300R": 438,
    "Pylon1300BR": 432,
    "Pylon1300BExcelon": 430,
    "Pylon1300BRExcelon": 433,
    "Pylon2048Series": 423,
    "Pylon2048F": 415,
    "Pylon2048B": 434,
    "Pylon2048BR": 416,
    "Pylon2048BExcelon": 435,
    "Pylon2048BRExcelon": 436,
    "Pylon2KSeries": 424,
    "Pylon2KF": 413,
    "Pylon2KB": 414,
    "Pylon2KBUV": 437,
    "Pylon2KBExcelon": 431,
    "PylonirSeries": 900,
    "Pylonir1024Series": 901,
    "Pylonir102422": 902,
    "Pylonir102417": 903,
    "PionirSeries": 500,
    "Pionir640": 501,
    "NirvanaSeries": 800,
    "Nirvana640": 801,
    "NirvanaSTSeries": 1300,
    "NirvanaST640": 1301,
    "NirvanaLNSeries": 1100,
    "NirvanaLN640": 1101
}
PicamModelLookup = dict(zip(PicamModel.values(), PicamModel.keys()))

PicamComputerInterface = {
    "Usb2": 1,
    "1394A": 2,
    "GigabitEthernet": 3
}
PicamComputerInterfaceLookup = dict(zip(PicamComputerInterface.values(), PicamComputerInterface.keys()))

PicamStringSize = {
    "SensorName": 64,
    "SerialNumber": 64,
    "FirmwareName": 64,
    "FirmwareDetail": 256
}
PicamStringSizeLookup = dict(zip(PicamStringSize.values(), PicamStringSize.keys()))

PicamValueType = {
    "Integer": 1,
    "Boolean": 3,
    "Enumeration": 4,
    "LargeInteger": 6,
    "FloatingPoint": 2,
    "Rois": 5,
    "Pulse": 7,
    "Modulations": 8
}
PicamValueTypeLookup = dict(zip(PicamValueType.values(), PicamValueType.keys()))

PicamConstraintType = {
    "None": 1,
    "Range": 2,
    "Collection": 3,
    "Rois": 4,
    "Pulse": 5,
    "Modulations": 6
}
PicamConstraintTypeLookup = dict(zip(PicamConstraintType.values(), PicamConstraintType.keys()))

PI_V = lambda v, c, n: (PicamConstraintType[c] << 24) + (PicamValueType[v] << 16) + n

PicamParameter = {
    "ExposureTime": PI_V("FloatingPoint", "Range", 23),
    "ShutterTimingMode": PI_V("Enumeration", "Collection", 24),
    "ShutterOpeningDelay": PI_V("FloatingPoint", "Range", 46),
    "ShutterClosingDelay": PI_V("FloatingPoint", "Range", 25),
    "ShutterDelayResolution": PI_V("FloatingPoint", "Collection", 47),
    "EnableIntensifier": PI_V("Boolean", "Collection", 86),
    "IntensifierStatus": PI_V("Enumeration", "None", 87),
    "IntensifierGain": PI_V("Integer", "Range", 88),
    "EMIccdGainControlMode": PI_V("Enumeration", "Collection", 123),
    "EMIccdGain": PI_V("Integer", "Range", 124),
    "PhosphorDecayDelay": PI_V("FloatingPoint", "Range", 89),
    "PhosphorDecayDelayResolution": PI_V("FloatingPoint", "Collection", 90),
    "GatingMode": PI_V("Enumeration", "Collection", 93),
    "RepetitiveGate": PI_V("Pulse", "Pulse", 94),
    "SequentialStartingGate": PI_V("Pulse", "Pulse", 95),
    "SequentialEndingGate": PI_V("Pulse", "Pulse", 96),
    "SequentialGateStepCount": PI_V("LargeInteger", "Range", 97),
    "SequentialGateStepIterations": PI_V("LargeInteger", "Range", 98),
    "DifStartingGate": PI_V("Pulse", "Pulse", 102),
    "DifEndingGate": PI_V("Pulse", "Pulse", 103),
    "BracketGating": PI_V("Boolean", "Collection", 100),
    "IntensifierOptions": PI_V("Enumeration", "None", 101),
    "EnableModulation": PI_V("Boolean", "Collection", 111),
    "ModulationDuration": PI_V("FloatingPoint", "Range", 118),
    "ModulationFrequency": PI_V("FloatingPoint", "Range", 112),
    "RepetitiveModulationPhase": PI_V("FloatingPoint", "Range", 113),
    "SequentialStartingModulationPhase": PI_V("FloatingPoint", "Range", 114),
    "SequentialEndingModulationPhase": PI_V("FloatingPoint", "Range", 115),
    "CustomModulationSequence": PI_V("Modulations", "Modulations", 119),
    "PhotocathodeSensitivity": PI_V("Enumeration", "None", 107),
    "GatingSpeed": PI_V("Enumeration", "None", 108),
    "PhosphorType": PI_V("Enumeration", "None", 109),
    "IntensifierDiameter": PI_V("FloatingPoint", "None", 110),
    "AdcSpeed": PI_V("FloatingPoint", "Collection", 33),
    "AdcBitDepth": PI_V("Integer", "Collection", 34),
    "AdcAnalogGain": PI_V("Enumeration", "Collection", 35),
    "AdcQuality": PI_V("Enumeration", "Collection", 36),
    "AdcEMGain": PI_V("Integer", "Range", 53),
    "CorrectPixelBias": PI_V("Boolean", "Collection", 106),
    "TriggerSource": PI_V("Enumeration", "Collection", 79),
    "TriggerResponse": PI_V("Enumeration", "Collection", 30),
    "TriggerDetermination": PI_V("Enumeration", "Collection", 31),
    "TriggerFrequency": PI_V("FloatingPoint", "Range", 80),
    "TriggerTermination": PI_V("Enumeration", "Collection", 81),
    "TriggerCoupling": PI_V("Enumeration", "Collection", 82),
    "TriggerThreshold": PI_V("FloatingPoint", "Range", 83),
    "OutputSignal": PI_V("Enumeration", "Collection", 32),
    "InvertOutputSignal": PI_V("Boolean", "Collection", 52),
    "AuxOutput": PI_V("Pulse", "Pulse", 91),
    "EnableSyncMaster": PI_V("Boolean", "Collection", 84),
    "SyncMaster2Delay": PI_V("FloatingPoint", "Range", 85),
    "EnableModulationOutputSignal": PI_V("Boolean", "Collection", 116),
    "ModulationOutputSignalFrequency": PI_V("FloatingPoint", "Range", 117),
    "ModulationOutputSignalAmplitude": PI_V("FloatingPoint", "Range", 120),
    "ReadoutControlMode": PI_V("Enumeration", "Collection", 26),
    "ReadoutTimeCalculation": PI_V("FloatingPoint", "None", 27),
    "ReadoutPortCount": PI_V("Integer", "Collection", 28),
    "ReadoutOrientation": PI_V("Enumeration", "None", 54),
    "KineticsWindowHeight": PI_V("Integer", "Range", 56),
    "VerticalShiftRate": PI_V("FloatingPoint", "Collection", 13),
    "Accumulations": PI_V("LargeInteger", "Range", 92),
    "EnableNondestructiveReadout": PI_V("Boolean", "Collection", 128),
    "NondestructiveReadoutPeriod": PI_V("FloatingPoint", "Range", 129),
    "Rois": PI_V("Rois", "Rois", 37),
    "NormalizeOrientation": PI_V("Boolean", "Collection", 39),
    "DisableDataFormatting": PI_V("Boolean", "Collection", 55),
    "ReadoutCount": PI_V("LargeInteger", "Range", 40),
    "ExactReadoutCountMaximum": PI_V("LargeInteger", "None", 77),
    "PhotonDetectionMode": PI_V("Enumeration", "Collection", 125),
    "PhotonDetectionThreshold": PI_V("FloatingPoint", "Range", 126),
    "PixelFormat": PI_V("Enumeration", "Collection", 41),
    "FrameSize": PI_V("Integer", "None", 42),
    "FrameStride": PI_V("Integer", "None", 43),
    "FramesPerReadout": PI_V("Integer", "None", 44),
    "ReadoutStride": PI_V("Integer", "None", 45),
    "PixelBitDepth": PI_V("Integer", "None", 48),
    "ReadoutRateCalculation": PI_V("FloatingPoint", "None", 50),
    "OnlineReadoutRateCalculation": PI_V("FloatingPoint", "None", 99),
    "FrameRateCalculation": PI_V("FloatingPoint", "None", 51),
    "Orientation": PI_V("Enumeration", "None", 38),
    "TimeStamps": PI_V("Enumeration", "Collection", 68),
    "TimeStampResolution": PI_V("LargeInteger", "Collection", 69),
    "TimeStampBitDepth": PI_V("Integer", "Collection", 70),
    "TrackFrames": PI_V("Boolean", "Collection", 71),
    "FrameTrackingBitDepth": PI_V("Integer", "Collection", 72),
    "GateTracking": PI_V("Enumeration", "Collection", 104),
    "GateTrackingBitDepth": PI_V("Integer", "Collection", 105),
    "ModulationTracking": PI_V("Enumeration", "Collection", 121),
    "ModulationTrackingBitDepth": PI_V("Integer", "Collection", 122),
    "SensorType": PI_V("Enumeration", "None", 57),
    "CcdCharacteristics": PI_V("Enumeration", "None", 58),
    "SensorActiveWidth": PI_V("Integer", "None", 59),
    "SensorActiveHeight": PI_V("Integer", "None", 60),
    "SensorActiveLeftMargin": PI_V("Integer", "None", 61),
    "SensorActiveTopMargin": PI_V("Integer", "None", 62),
    "SensorActiveRightMargin": PI_V("Integer", "None", 63),
    "SensorActiveBottomMargin": PI_V("Integer", "None", 64),
    "SensorMaskedHeight": PI_V("Integer", "None", 65),
    "SensorMaskedTopMargin": PI_V("Integer", "None", 66),
    "SensorMaskedBottomMargin": PI_V("Integer", "None", 67),
    "SensorSecondaryMaskedHeight": PI_V("Integer", "None", 49),
    "SensorSecondaryActiveHeight": PI_V("Integer", "None", 74),
    "PixelWidth": PI_V("FloatingPoint", "None", 9),
    "PixelHeight": PI_V("FloatingPoint", "None", 10),
    "PixelGapWidth": PI_V("FloatingPoint", "None", 11),
    "PixelGapHeight": PI_V("FloatingPoint", "None", 12),
    "ActiveWidth": PI_V("Integer", "Range", 1),
    "ActiveHeight": PI_V("Integer", "Range", 2),
    "ActiveLeftMargin": PI_V("Integer", "Range", 3),
    "ActiveTopMargin": PI_V("Integer", "Range", 4),
    "ActiveRightMargin": PI_V("Integer", "Range", 5),
    "ActiveBottomMargin": PI_V("Integer", "Range", 6),
    "MaskedHeight": PI_V("Integer", "Range", 7),
    "MaskedTopMargin": PI_V("Integer", "Range", 8),
    "MaskedBottomMargin": PI_V("Integer", "Range", 73),
    "SecondaryMaskedHeight": PI_V("Integer", "Range", 75),
    "SecondaryActiveHeight": PI_V("Integer", "Range", 76),
    "CleanSectionFinalHeight": PI_V("Integer", "Range", 17),
    "CleanSectionFinalHeightCount": PI_V("Integer", "Range", 18),
    "CleanSerialRegister": PI_V("Boolean", "Collection", 19),
    "CleanCycleCount": PI_V("Integer", "Range", 20),
    "CleanCycleHeight": PI_V("Integer", "Range", 21),
    "CleanBeforeExposure": PI_V("Boolean", "Collection", 78),
    "CleanUntilTrigger": PI_V("Boolean", "Collection", 22),
    "SensorTemperatureSetPoint": PI_V("FloatingPoint", "Range", 14),
    "SensorTemperatureReading": PI_V("FloatingPoint", "None", 15),
    "SensorTemperatureStatus": PI_V("Enumeration", "None", 16),
    "DisableCoolingFan": PI_V("Boolean", "Collection", 29),
    "EnableSensorWindowHeater": PI_V("Boolean", "Collection", 127)
}
PicamParameterLookup = dict(zip(PicamParameter.values(), PicamParameter.keys()))

PicamAdcAnalogGain = {
    "Low": 1,
    "Medium": 2,
    "High": 3
}
PicamAdcAnalogGainLookup = dict(zip(PicamAdcAnalogGain.values(), PicamAdcAnalogGain.keys()))

PicamAdcQuality = {
    "LowNoise": 1,
    "HighCapacity": 2,
    "HighSpeed": 4,
    "ElectronMultiplied": 3
}
PicamAdcQualityLookup = dict(zip(PicamAdcQuality.values(), PicamAdcQuality.keys()))

PicamCcdCharacteristicsMask = {
    "None": 0x000,
    "BackIlluminated": 0x001,
    "DeepDepleted": 0x002,
    "OpenElectrode": 0x004,
    "UVEnhanced": 0x008,
    "ExcelonEnabled": 0x010,
    "SecondaryMask": 0x020,
    "Multiport": 0x040,
    "AdvancedInvertedMode": 0x080,
    "HighResistivity": 0x100
}

PicamEMIccdGainControlMode = {
    "Optimal": 1,
    "Manual": 2
}
PicamEMIccdGainControlModeLookup = dict(zip(PicamEMIccdGainControlMode.values(), PicamEMIccdGainControlMode.keys()))

PicamGateTrackingMask = {
    "None": 0x0,
    "Delay": 0x1,
    "Width": 0x2
}
PicamGateTrackingMaskLookup = dict(zip(PicamGateTrackingMask.values(), PicamGateTrackingMask.keys()))

PicamGatingMode = {
    "Repetitive": 1,
    "Sequential": 2,
    "Custom": 3
}
PicamGatingModeLookup = dict(zip(PicamGatingMode.values(), PicamGatingMode.keys()))

PicamGatingSpeed = {
    "Fast": 1,
    "Slow": 2
}
PicamGatingSpeedLookup = dict(zip(PicamGatingSpeed.values(), PicamGatingSpeed.keys()))

PicamIntensifierOptionsMask = {
    "None": 0x0,
    "McpGating": 0x1,
    "SubNanosecondGating": 0x2,
    "Modulation": 0x4
}
PicamIntensifierOptionsMaskLookup = dict(zip(PicamIntensifierOptionsMask.values(), PicamIntensifierOptionsMask.keys()))

PicamIntensifierStatus = {
    "PoweredOff": 1,
    "PoweredOn": 2
}
PicamIntensifierStatusLookup = dict(zip(PicamIntensifierStatus.values(), PicamIntensifierStatus.keys()))

PicamModulationTrackingMask = {
    "None": 0x0,
    "Duration": 0x1,
    "Frequency": 0x2,
    "Phase": 0x4,
    "OutputSignalFrequency": 0x8
}
PicamModulationTrackingMaskLookup = dict(zip(PicamModulationTrackingMask.values(), PicamModulationTrackingMask.keys()))

PicamOrientationMask = {
    "Normal": 0x0,
    "FlippedHorizontally": 0x1,
    "FlippedVertically": 0x2
}
PicamOrientationMaskLookup = dict(zip(PicamOrientationMask.values(), PicamOrientationMask.keys()))

PicamOutputSignal = {
    "NotReadingOut": 1,
    "ShutterOpen": 2,
    "Busy": 3,
    "AlwaysLow": 4,
    "AlwaysHigh": 5,
    "Acquiring": 6,
    "ShiftingUnderMask": 7,
    "Exposing": 8,
    "EffectivelyExposing": 9,
    "ReadingOut": 10,
    "WaitingForTrigger": 11
}
PicamOutputSignalLookup = dict(zip(PicamOutputSignal.values(), PicamOutputSignal.keys()))

PicamPhosphorType = {
    "P43": 1,
    "P46": 2
}
PicamPhosphorTypeLookup = dict(zip(PicamPhosphorType.values(), PicamPhosphorType.keys()))

PicamPhotocathodeSensitivity = {
    "RedBlue": 1,
    "SuperRed": 7,
    "SuperBlue": 2,
    "UV": 3,
    "SolarBlind": 10,
    "Unigen2Filmless": 4,
    "InGaAsFilmless": 9,
    "HighQEFilmless": 5,
    "HighRedFilmless": 8,
    "HighBlueFilmless": 6
}
PicamPhotocathodeSensitivityLookup = dict(zip(PicamPhotocathodeSensitivity.values(), PicamPhotocathodeSensitivity.keys()))

PicamPhotonDetectionMode = {
    "Disabled": 1,
    "Thresholding": 2,
    "Clipping": 3
}
PicamPhotonDetectionModeLookup = dict(zip(PicamPhotonDetectionMode.values(), PicamPhotonDetectionMode.keys()))

PicamPixelFormat = {
    "Monochrome16Bit": 1
}
PicamPixelFormatLookup = dict(zip(PicamPixelFormat.values(), PicamPixelFormat.keys()))

PicamReadoutControlMode = {
    "FullFrame": 1,
    "FrameTransfer": 2,
    "Interline": 5,
    "Kinetics": 3,
    "SpectraKinetics": 4,
    "Dif": 6
}
PicamReadoutControlModeLookup = dict(zip(PicamReadoutControlMode.values(), PicamReadoutControlMode.keys()))

PicamSensorTemperatureStatus = {
    "Unlocked": 1,
    "Locked": 2
}
PicamSensorTemperatureStatusLookup = dict(zip(PicamSensorTemperatureStatus.values(), PicamSensorTemperatureStatus.keys()))

PicamSensorType = {
    "Ccd": 1,
    "InGaAs": 2
}
PicamSensorTypeLookup = dict(zip(PicamSensorType.values(), PicamSensorType.keys()))

PicamShutterTimingMode = {
    "Normal": 1,
    "AlwaysClosed": 2,
    "AlwaysOpen": 3,
    "OpenBeforeTrigger": 4
}
PicamShutterTimingModeLookup = dict(zip(PicamShutterTimingMode.values(), PicamShutterTimingMode.keys()))

PicamTimeStampsMask = {
    "None": 0x0,
    "ExposureStarted": 0x1,
    "ExposureEnded": 0x2
}
PicamTimeStampsMaskLookup = dict(zip(PicamTimeStampsMask.values(), PicamTimeStampsMask.keys()))

PicamTriggerCoupling = {
    "AC": 1,
    "DC": 2
}
PicamTriggerCouplingLookup = dict(zip(PicamTriggerCoupling.values(), PicamTriggerCoupling.keys()))

PicamTriggerDetermination = {
    "PositivePolarity": 1,
    "NegativePolarity": 2,
    "RisingEdge": 3,
    "FallingEdge": 4
}
PicamTriggerDeterminationLookup = dict(zip(PicamTriggerDetermination.values(), PicamTriggerDetermination.keys()))

PicamTriggerResponse = {
    "NoResponse": 1,
    "ReadoutPerTrigger": 2,
    "ShiftPerTrigger": 3,
    "ExposeDuringTriggerPulse": 4,
    "StartOnSingleTrigger": 5
}
PicamTriggerResponseLookup = dict(zip(PicamTriggerResponse.values(), PicamTriggerResponse.keys()))

PicamTriggerSource = {
    "External": 1,
    "Internal": 2
}
PicamTriggerSourceLookup = dict(zip(PicamTriggerSource.values(), PicamTriggerSource.keys()))

PicamTriggerTermination = {
    "FiftyOhms": 1,
    "HighImpedance": 2
}
PicamTriggerTerminationLookup = dict(zip(PicamTriggerTermination.values(), PicamTriggerTermination.keys()))

PicamValueAccess = {
    "ReadOnly": 1,
    "ReadWriteTrivial": 3,
    "ReadWrite": 2
}
PicamValueAccessLookup = dict(zip(PicamValueAccess.values(), PicamValueAccess.keys()))

PicamConstraintScope = {
    "Independent": 1,
    "Dependent": 2
}
PicamConstraintScopeLookup = dict(zip(PicamConstraintScope.values(), PicamConstraintScope.keys()))

PicamConstraintSeverity = {
    "Error": 1,
    "Warning": 2
}
PicamConstraintSeverityLookup = dict(zip(PicamConstraintSeverity.values(), PicamConstraintSeverity.keys()))

PicamConstraintCategory = {
    "Capable": 1,
    "Required": 2,
    "Recommended": 3
}
PicamConstraintSeverityLookup = dict(zip(PicamConstraintSeverity.values(), PicamConstraintSeverity.keys()))

PicamRoisConstraintRulesMask = {
    "None": 0x00,
    "XBinningAlignment": 0x01,
    "YBinningAlignment": 0x02,
    "HorizontalSymmetry": 0x04,
    "VerticalSymmetry": 0x08,
    "SymmetryBoundsBinning": 0x10
}
PicamRoisConstraintRulesMaskLookup = dict(zip(PicamRoisConstraintRulesMask.values(), PicamRoisConstraintRulesMask.keys()))

PicamAcquisitionErrorsMask = {
    "None": 0x0,
    "DataLost": 0x1,
    "ConnectionLost": 0x2
}
PicamAcquisitionErrorsMaskLookup = dict(zip(PicamAcquisitionErrorsMask.values(), PicamAcquisitionErrorsMask.keys()))


# +++++++ structures +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class PicamCameraID(ctypes.Structure):
        _fields_ = [("model", piint),
                    ("computer_interface", piint),
                    ("sensor_name", pichar * PicamStringSize["SensorName"]),
                    ("serial_number", pichar * PicamStringSize["SerialNumber"])]


class PicamFirmwareDetail(ctypes.Structure):
    _fields_ = [("name", pichar * PicamStringSize["FirmwareName"]),
                ("detail", pichar * PicamStringSize["FirmwareDetail"])]


class PicamRoi(ctypes.Structure):
    _fields_ = [("x", piint),
                ("width", piint),
                ("x_binning", piint),
                ("y", piint),
                ("height", piint),
                ("y_binning", piint)]


class PicamRois(ctypes.Structure):
    _fields_ = [("roi_array", ctypes.POINTER(PicamRoi)),
                ("roi_count", piint)]


class PicamPulse(ctypes.Structure):
    _fields_ = [("delay", piflt),
                ("width", piflt)]


class PicamModulation(ctypes.Structure):
    _fields_ = [("duration", piflt),
                ("frequency", piflt),
                ("phase", piflt),
                ("output_signal_frequency", piflt)]


class PicamModulations(ctypes.Structure):
    _fields_ = [("modulation_array", ctypes.POINTER(PicamModulation)),
                ("modulation_count", piint)]


class PicamCollectionConstraint(ctypes.Structure):
    _fields_ = [("scope", piint),
                ("severity", piint),
                ("values_array", ctypes.POINTER(piflt)),
                ("values_count", piint)]


class PicamRangeConstraint(ctypes.Structure):
    _fields_ = [("scope", piint),
                ("severity", piint),
                ("empty_set", pibln),
                ("minimum", piflt),
                ("maximum", piflt),
                ("increment", piflt),
                ("excluded_values_array", ctypes.POINTER(piflt)),
                ("excluded_values_count", piint),
                ("outlying_values_array", ctypes.POINTER(piflt)),
                ("outlying_values_count", piint)]


class PicamRoisConstraint(ctypes.Structure):
    _fields_ = [("scope", piint),
                ("severity", piint),
                ("empty_set", pibln),
                ("rules", piint),
                ("maximum_roi_count", piint),
                ("x_constraint", PicamRangeConstraint),
                ("width_constraint", PicamRangeConstraint),
                ("x_binning_limits_array", ctypes.POINTER(piint)),
                ("x_binning_limits_count", piint),
                ("y_constraint", PicamRangeConstraint),
                ("height_constraint", PicamRangeConstraint),
                ("y_binning_limits_array", ctypes.POINTER(piint)),
                ("y_binning_limits_count", piint)]


class PicamPulseConstraint(ctypes.Structure):
    _fields_ = [("scope", piint),
                ("severity", piint),
                ("empty_set", pibln),
                ("delay_constraint", PicamRangeConstraint),
                ("width_constraint", PicamRangeConstraint),
                ("minimum_duration", piflt),
                ("maximum_duration", piflt)]


class PicamModulationsConstraint(ctypes.Structure):
    _fields_ = [("scope", piint),
                ("severity", piint),
                ("empty_set", pibln),
                ("maximum_modulation_count", piint),
                ("duration_constraint", PicamRangeConstraint),
                ("frequency_constraint", PicamRangeConstraint),
                ("phase_constraint", PicamRangeConstraint),
                ("output_signal_frequency_constraint", PicamRangeConstraint)]


class PicamAvailableData(ctypes.Structure):
    _fields_ = [("initial_readout", pivoid),
                ("readout_count", pi64s)]


class PicamAcquisitionStatus(ctypes.Structure):
    _fields_ = [("running", pibln),
                ("errors", piint),
                ("readout_rate", piflt)]
