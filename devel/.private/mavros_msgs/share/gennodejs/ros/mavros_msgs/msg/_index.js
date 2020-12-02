
"use strict";

let RTCM = require('./RTCM.js');
let ActuatorControl = require('./ActuatorControl.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let PositionTarget = require('./PositionTarget.js');
let HilSensor = require('./HilSensor.js');
let Param = require('./Param.js');
let Mavlink = require('./Mavlink.js');
let FileEntry = require('./FileEntry.js');
let RTKBaseline = require('./RTKBaseline.js');
let ExtendedState = require('./ExtendedState.js');
let RadioStatus = require('./RadioStatus.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let CommandCode = require('./CommandCode.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let RCOut = require('./RCOut.js');
let LandingTarget = require('./LandingTarget.js');
let Trajectory = require('./Trajectory.js');
let ManualControl = require('./ManualControl.js');
let Altitude = require('./Altitude.js');
let WaypointReached = require('./WaypointReached.js');
let GPSRTK = require('./GPSRTK.js');
let DebugValue = require('./DebugValue.js');
let VFR_HUD = require('./VFR_HUD.js');
let ESCStatus = require('./ESCStatus.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let Thrust = require('./Thrust.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let MountControl = require('./MountControl.js');
let Vibration = require('./Vibration.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let MotorSetpoint = require('./MotorSetpoint.js');
let WaypointList = require('./WaypointList.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let BatteryStatus = require('./BatteryStatus.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let LogEntry = require('./LogEntry.js');
let LogData = require('./LogData.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let StatusText = require('./StatusText.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let GPSRAW = require('./GPSRAW.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let RCIn = require('./RCIn.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let Waypoint = require('./Waypoint.js');
let VehicleInfo = require('./VehicleInfo.js');
let HilControls = require('./HilControls.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let ESCInfo = require('./ESCInfo.js');
let HomePosition = require('./HomePosition.js');
let ParamValue = require('./ParamValue.js');
let State = require('./State.js');
let HilGPS = require('./HilGPS.js');

module.exports = {
  RTCM: RTCM,
  ActuatorControl: ActuatorControl,
  OnboardComputerStatus: OnboardComputerStatus,
  PositionTarget: PositionTarget,
  HilSensor: HilSensor,
  Param: Param,
  Mavlink: Mavlink,
  FileEntry: FileEntry,
  RTKBaseline: RTKBaseline,
  ExtendedState: ExtendedState,
  RadioStatus: RadioStatus,
  CamIMUStamp: CamIMUStamp,
  CommandCode: CommandCode,
  AttitudeTarget: AttitudeTarget,
  RCOut: RCOut,
  LandingTarget: LandingTarget,
  Trajectory: Trajectory,
  ManualControl: ManualControl,
  Altitude: Altitude,
  WaypointReached: WaypointReached,
  GPSRTK: GPSRTK,
  DebugValue: DebugValue,
  VFR_HUD: VFR_HUD,
  ESCStatus: ESCStatus,
  ESCStatusItem: ESCStatusItem,
  GlobalPositionTarget: GlobalPositionTarget,
  HilActuatorControls: HilActuatorControls,
  Thrust: Thrust,
  EstimatorStatus: EstimatorStatus,
  MountControl: MountControl,
  Vibration: Vibration,
  OpticalFlowRad: OpticalFlowRad,
  MotorSetpoint: MotorSetpoint,
  WaypointList: WaypointList,
  ESCInfoItem: ESCInfoItem,
  BatteryStatus: BatteryStatus,
  CompanionProcessStatus: CompanionProcessStatus,
  TimesyncStatus: TimesyncStatus,
  LogEntry: LogEntry,
  LogData: LogData,
  OverrideRCIn: OverrideRCIn,
  StatusText: StatusText,
  ADSBVehicle: ADSBVehicle,
  GPSRAW: GPSRAW,
  PlayTuneV2: PlayTuneV2,
  RCIn: RCIn,
  HilStateQuaternion: HilStateQuaternion,
  Waypoint: Waypoint,
  VehicleInfo: VehicleInfo,
  HilControls: HilControls,
  WheelOdomStamped: WheelOdomStamped,
  ESCInfo: ESCInfo,
  HomePosition: HomePosition,
  ParamValue: ParamValue,
  State: State,
  HilGPS: HilGPS,
};
