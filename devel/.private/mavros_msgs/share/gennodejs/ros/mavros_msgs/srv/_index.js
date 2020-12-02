
"use strict";

let CommandInt = require('./CommandInt.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let WaypointPush = require('./WaypointPush.js')
let FileWrite = require('./FileWrite.js')
let LogRequestData = require('./LogRequestData.js')
let ParamPull = require('./ParamPull.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let SetMode = require('./SetMode.js')
let SetMavFrame = require('./SetMavFrame.js')
let FileClose = require('./FileClose.js')
let FileList = require('./FileList.js')
let CommandTOL = require('./CommandTOL.js')
let ParamPush = require('./ParamPush.js')
let CommandHome = require('./CommandHome.js')
let WaypointClear = require('./WaypointClear.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let LogRequestList = require('./LogRequestList.js')
let CommandBool = require('./CommandBool.js')
let FileRename = require('./FileRename.js')
let ParamGet = require('./ParamGet.js')
let FileRead = require('./FileRead.js')
let FileChecksum = require('./FileChecksum.js')
let FileMakeDir = require('./FileMakeDir.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let CommandLong = require('./CommandLong.js')
let WaypointPull = require('./WaypointPull.js')
let FileTruncate = require('./FileTruncate.js')
let FileRemove = require('./FileRemove.js')
let MountConfigure = require('./MountConfigure.js')
let MessageInterval = require('./MessageInterval.js')
let FileOpen = require('./FileOpen.js')
let ParamSet = require('./ParamSet.js')
let StreamRate = require('./StreamRate.js')

module.exports = {
  CommandInt: CommandInt,
  CommandTriggerControl: CommandTriggerControl,
  CommandVtolTransition: CommandVtolTransition,
  WaypointPush: WaypointPush,
  FileWrite: FileWrite,
  LogRequestData: LogRequestData,
  ParamPull: ParamPull,
  CommandTriggerInterval: CommandTriggerInterval,
  SetMode: SetMode,
  SetMavFrame: SetMavFrame,
  FileClose: FileClose,
  FileList: FileList,
  CommandTOL: CommandTOL,
  ParamPush: ParamPush,
  CommandHome: CommandHome,
  WaypointClear: WaypointClear,
  LogRequestEnd: LogRequestEnd,
  WaypointSetCurrent: WaypointSetCurrent,
  FileRemoveDir: FileRemoveDir,
  LogRequestList: LogRequestList,
  CommandBool: CommandBool,
  FileRename: FileRename,
  ParamGet: ParamGet,
  FileRead: FileRead,
  FileChecksum: FileChecksum,
  FileMakeDir: FileMakeDir,
  VehicleInfoGet: VehicleInfoGet,
  CommandLong: CommandLong,
  WaypointPull: WaypointPull,
  FileTruncate: FileTruncate,
  FileRemove: FileRemove,
  MountConfigure: MountConfigure,
  MessageInterval: MessageInterval,
  FileOpen: FileOpen,
  ParamSet: ParamSet,
  StreamRate: StreamRate,
};
