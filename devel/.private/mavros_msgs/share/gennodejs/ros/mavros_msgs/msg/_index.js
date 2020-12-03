
"use strict";

let State = require('./State.js');
let MotorSetpoint = require('./MotorSetpoint.js');
let StatusText = require('./StatusText.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let RCIn = require('./RCIn.js');
let LogData = require('./LogData.js');
let WaypointList = require('./WaypointList.js');
let HilControls = require('./HilControls.js');
let ESCStatus = require('./ESCStatus.js');
let ParamValue = require('./ParamValue.js');
let Trajectory = require('./Trajectory.js');
let VehicleInfo = require('./VehicleInfo.js');
let ManualControl = require('./ManualControl.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let ExtendedState = require('./ExtendedState.js');
let RCOut = require('./RCOut.js');
let CommandCode = require('./CommandCode.js');
let Vibration = require('./Vibration.js');
let RTCM = require('./RTCM.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let Altitude = require('./Altitude.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let Thrust = require('./Thrust.js');
let HomePosition = require('./HomePosition.js');
let ESCInfo = require('./ESCInfo.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let Param = require('./Param.js');
let Mavlink = require('./Mavlink.js');
let HilSensor = require('./HilSensor.js');
let WaypointReached = require('./WaypointReached.js');
let LogEntry = require('./LogEntry.js');
let HilGPS = require('./HilGPS.js');
let FileEntry = require('./FileEntry.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let DebugValue = require('./DebugValue.js');
let VFR_HUD = require('./VFR_HUD.js');
let RadioStatus = require('./RadioStatus.js');
let MountControl = require('./MountControl.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let LandingTarget = require('./LandingTarget.js');
let BatteryStatus = require('./BatteryStatus.js');
let GPSRTK = require('./GPSRTK.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let ActuatorControl = require('./ActuatorControl.js');
let RTKBaseline = require('./RTKBaseline.js');
let GPSRAW = require('./GPSRAW.js');
let PositionTarget = require('./PositionTarget.js');
let Waypoint = require('./Waypoint.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let OverrideRCIn = require('./OverrideRCIn.js');

module.exports = {
  State: State,
  MotorSetpoint: MotorSetpoint,
  StatusText: StatusText,
  HilActuatorControls: HilActuatorControls,
  RCIn: RCIn,
  LogData: LogData,
  WaypointList: WaypointList,
  HilControls: HilControls,
  ESCStatus: ESCStatus,
  ParamValue: ParamValue,
  Trajectory: Trajectory,
  VehicleInfo: VehicleInfo,
  ManualControl: ManualControl,
  ADSBVehicle: ADSBVehicle,
  CamIMUStamp: CamIMUStamp,
  ExtendedState: ExtendedState,
  RCOut: RCOut,
  CommandCode: CommandCode,
  Vibration: Vibration,
  RTCM: RTCM,
  GlobalPositionTarget: GlobalPositionTarget,
  OnboardComputerStatus: OnboardComputerStatus,
  Altitude: Altitude,
  ESCStatusItem: ESCStatusItem,
  Thrust: Thrust,
  HomePosition: HomePosition,
  ESCInfo: ESCInfo,
  AttitudeTarget: AttitudeTarget,
  CompanionProcessStatus: CompanionProcessStatus,
  Param: Param,
  Mavlink: Mavlink,
  HilSensor: HilSensor,
  WaypointReached: WaypointReached,
  LogEntry: LogEntry,
  HilGPS: HilGPS,
  FileEntry: FileEntry,
  OpticalFlowRad: OpticalFlowRad,
  HilStateQuaternion: HilStateQuaternion,
  DebugValue: DebugValue,
  VFR_HUD: VFR_HUD,
  RadioStatus: RadioStatus,
  MountControl: MountControl,
  EstimatorStatus: EstimatorStatus,
  LandingTarget: LandingTarget,
  BatteryStatus: BatteryStatus,
  GPSRTK: GPSRTK,
  ESCInfoItem: ESCInfoItem,
  PlayTuneV2: PlayTuneV2,
  ActuatorControl: ActuatorControl,
  RTKBaseline: RTKBaseline,
  GPSRAW: GPSRAW,
  PositionTarget: PositionTarget,
  Waypoint: Waypoint,
  WheelOdomStamped: WheelOdomStamped,
  TimesyncStatus: TimesyncStatus,
  OverrideRCIn: OverrideRCIn,
};
